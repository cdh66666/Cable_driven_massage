#!/usr/bin/env python3
"""Inspect a STEP/XCAF assembly without opening a CAD GUI.

The script prints one JSON object per assembly/component.  It is intentionally
small and dependency-light; it only needs OpenCascade's Python bindings (OCP).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_SurfaceType
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool


def label_name(label: TDF_Label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def shape_stats(shape) -> dict:
    if shape.IsNull():
        return {"null": True}

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, True)
    stats: dict[str, object] = {}
    if not box.IsVoid():
        xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
        stats["bbox"] = [round(v, 6) for v in (xmin, ymin, zmin, xmax, ymax, zmax)]
        stats["size"] = [round(xmax - xmin, 6), round(ymax - ymin, 6), round(zmax - zmin, 6)]
        stats["center"] = [round((xmin + xmax) / 2, 6), round((ymin + ymax) / 2, 6), round((zmin + zmax) / 2, 6)]

    for key, kind in (("solids", TopAbs_SOLID), ("faces", TopAbs_FACE), ("edges", TopAbs_EDGE)):
        count = 0
        explorer = TopExp_Explorer(shape, kind)
        while explorer.More():
            count += 1
            explorer.Next()
        stats[key] = count

    props = GProp_GProps()
    try:
        BRepGProp.VolumeProperties_s(shape, props)
        stats["volume"] = round(props.Mass(), 6)
        c = props.CentreOfMass()
        stats["centroid"] = [round(c.X(), 6), round(c.Y(), 6), round(c.Z(), 6)]
    except Exception:
        pass
    return stats


def referred_label(label: TDF_Label) -> TDF_Label | None:
    if not XCAFDoc_ShapeTool.IsReference_s(label):
        return None
    referred = TDF_Label()
    if XCAFDoc_ShapeTool.GetReferredShape_s(label, referred):
        return referred
    return None


def component_children(label: TDF_Label) -> list[TDF_Label]:
    source = referred_label(label) or label
    if not XCAFDoc_ShapeTool.IsAssembly_s(source):
        return []
    labels = TDF_LabelSequence()
    XCAFDoc_ShapeTool.GetComponents_s(source, labels, False)
    return [labels.Value(i) for i in range(1, labels.Length() + 1)]


def label_at_path(roots: TDF_LabelSequence, path: str) -> TDF_Label:
    indexes = [int(part) for part in path.split("/") if part]
    if not indexes:
        raise ValueError("empty component path")
    label = roots.Value(indexes[0])
    for index in indexes[1:]:
        children = component_children(label)
        label = children[index - 1]
    return label


def explode_solids(label: TDF_Label, path: str) -> None:
    shape = XCAFDoc_ShapeTool.GetShape_s(label)
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    index = 0
    while explorer.More():
        index += 1
        row = {"path": f"{path}#solid{index}", **shape_stats(explorer.Current())}
        print(json.dumps(row, ensure_ascii=False, separators=(",", ":")))
        explorer.Next()


def nth_solid(shape, wanted: int):
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    index = 0
    while explorer.More():
        index += 1
        if index == wanted:
            return explorer.Current()
        explorer.Next()
    raise IndexError(f"solid {wanted} does not exist")


def point_xyz(point) -> list[float]:
    return [round(point.X(), 6), round(point.Y(), 6), round(point.Z(), 6)]


def direction_xyz(direction) -> list[float]:
    return [round(direction.X(), 9), round(direction.Y(), 9), round(direction.Z(), 9)]


def face_surface_stats(face) -> dict:
    adaptor = BRepAdaptor_Surface(face, True)
    kind = adaptor.GetType()
    row: dict[str, object] = {"surface": kind.name.removeprefix("GeomAbs_")}
    if kind == GeomAbs_SurfaceType.GeomAbs_Plane:
        plane = adaptor.Plane()
        row.update(origin=point_xyz(plane.Location()), axis=direction_xyz(plane.Axis().Direction()))
    elif kind == GeomAbs_SurfaceType.GeomAbs_Cylinder:
        cylinder = adaptor.Cylinder()
        row.update(
            origin=point_xyz(cylinder.Location()),
            axis=direction_xyz(cylinder.Axis().Direction()),
            radius=round(cylinder.Radius(), 6),
        )
    elif kind == GeomAbs_SurfaceType.GeomAbs_Sphere:
        sphere = adaptor.Sphere()
        row.update(center=point_xyz(sphere.Location()), radius=round(sphere.Radius(), 6))
    elif kind == GeomAbs_SurfaceType.GeomAbs_Cone:
        cone = adaptor.Cone()
        row.update(
            origin=point_xyz(cone.Location()),
            axis=direction_xyz(cone.Axis().Direction()),
            ref_radius=round(cone.RefRadius(), 6),
            semi_angle=round(cone.SemiAngle(), 9),
        )
    elif kind == GeomAbs_SurfaceType.GeomAbs_Torus:
        torus = adaptor.Torus()
        row.update(
            origin=point_xyz(torus.Location()),
            axis=direction_xyz(torus.Axis().Direction()),
            major_radius=round(torus.MajorRadius(), 6),
            minor_radius=round(torus.MinorRadius(), 6),
        )
    area = GProp_GProps()
    try:
        BRepGProp.SurfaceProperties_s(face, area)
        row["area"] = round(area.Mass(), 6)
        row["area_centroid"] = point_xyz(area.CentreOfMass())
    except Exception:
        pass
    box = Bnd_Box()
    BRepBndLib.Add_s(face, box, True)
    if not box.IsVoid():
        row["bbox"] = [round(v, 6) for v in box.Get()]
    return row


def list_faces(label: TDF_Label, path: str, solid_index: int) -> None:
    shape = XCAFDoc_ShapeTool.GetShape_s(label)
    target = nth_solid(shape, solid_index)
    explorer = TopExp_Explorer(target, TopAbs_FACE)
    index = 0
    while explorer.More():
        index += 1
        face = TopoDS.Face_s(explorer.Current())
        row = {"path": f"{path}#solid{solid_index}/face{index}", **face_surface_stats(face)}
        print(json.dumps(row, ensure_ascii=False, separators=(",", ":")))
        explorer.Next()


def walk(label: TDF_Label, path: str, depth: int, max_depth: int) -> None:
    ref = referred_label(label)
    shape = XCAFDoc_ShapeTool.GetShape_s(label)
    row = {
        "path": path,
        "depth": depth,
        "name": label_name(label),
        "reference_name": label_name(ref) if ref is not None else "",
        "is_reference": ref is not None,
        "is_assembly": XCAFDoc_ShapeTool.IsAssembly_s(ref or label),
        **shape_stats(shape),
    }
    print(json.dumps(row, ensure_ascii=False, separators=(",", ":")))
    if depth >= max_depth:
        return
    for index, child in enumerate(component_children(label), 1):
        walk(child, f"{path}/{index}", depth + 1, max_depth)


def load_step(path: Path) -> TDocStd_Document:
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("cad-analysis"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(path))
    if int(status) != 1:
        raise RuntimeError(f"STEP read failed with status {status}")
    if not reader.Transfer(doc):
        raise RuntimeError("STEP transfer into XCAF document failed")
    return doc


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path)
    parser.add_argument("--max-depth", type=int, default=8)
    parser.add_argument(
        "--explode-solids-of",
        metavar="COMPONENT_PATH",
        help="only print the individual solids contained in this component path",
    )
    parser.add_argument(
        "--faces-of",
        metavar="COMPONENT_PATH",
        help="only print analytic faces from one solid in this component path",
    )
    parser.add_argument("--solid-index", type=int, default=1)
    args = parser.parse_args()

    doc = load_step(args.step.resolve())
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    if args.explode_solids_of:
        label = label_at_path(roots, args.explode_solids_of)
        explode_solids(label, args.explode_solids_of)
        return
    if args.faces_of:
        label = label_at_path(roots, args.faces_of)
        list_faces(label, args.faces_of, args.solid_index)
        return
    for index in range(1, roots.Length() + 1):
        walk(roots.Value(index), str(index), 0, args.max_depth)


if __name__ == "__main__":
    main()
