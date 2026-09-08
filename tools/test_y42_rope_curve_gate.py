"""AST-only regression for the CAD rope-curve guard: never import the server.

Only five pure production functions are compiled; literal constants and safe
stubs supply dependencies.  No serial, HTTP, persistence, or controller startup
can run.  The guard's own implementation remains in the main program.
"""

import ast
import copy
import math
from pathlib import Path
import statistics
import unittest


def isolated_curve_functions():
    source = Path(__file__).with_name("y42_pc_direct_web.py")
    tree = ast.parse(source.read_text(encoding="utf-8-sig"), filename=str(source))
    names = {
        "_coalesce_rope_curve", "rope_curve_quality_error", "calibrated_rope_curve",
        "internal_model_readiness", "_nominal_motor_degrees", "encoder_positions_to_lengths",
    }
    namespace = {"math": math, "statistics": statistics}
    # Evaluate literals only, never module statements or import side effects.
    for node in tree.body:
        if isinstance(node, ast.Assign):
            try:
                value = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
            for target in node.targets:
                if isinstance(target, ast.Name):
                    namespace[target.id] = value
    namespace.update({
        "PROCESS_SESSION_ID": "ast-test-session",
        "SPOOL_FLANGE_RADIUS_MM": 20.0,
        "SPOOL_MAX_PHYSICAL_RADIUS_MM": 20.0,
        "MOTOR_IDS": (1, 2, 3, 4),
        "calibration_data": lambda: {},
    })
    def forbidden_fallback(*args, **kwargs):
        raise AssertionError("Rejected curve fell through to a different runtime model")
    namespace.update({
        "workspace_anchor_coordinates": forbidden_fallback,
        "effective_slopes": forbidden_fallback,
        "interpolate_curve": forbidden_fallback,
        "invert_curve": forbidden_fallback,
    })
    selected = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    missing = names - {node.name for node in selected}
    if missing:
        raise AssertionError("Production pure guard is not implemented: " + ", ".join(sorted(missing)))
    module = ast.Module(body=selected, type_ignores=[])
    exec(compile(module, str(source) + " [isolated pure AST]", "exec"), namespace)
    return namespace


class RopeCurveGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = isolated_curve_functions()

    def good(self):
        return {
            "valid": True,
            "calibration_source": "torque_corner_initializer",
            "geometry_version": self.ns["GEOMETRY_VERSION"],
            "cable_exit_model": self.ns["CABLE_EXIT_MODEL"],
            "workspace_calibration_verified": True,
            "internal_model_verified": True,
            "session_calibration_mode": self.ns["SESSION_CALIBRATION_MODE"],
            "session_id": self.ns["PROCESS_SESSION_ID"],
            "session_local_verified": True,
            "session_tension_held": True,
            "slope": [5.0] * 4,
            "intercept": [0.0] * 4,
            "r2": [0.999] * 4,
            "distance_samples_mm": [[20.0, 80.0, 140.0, 180.0] for _ in range(4)],
            "position_samples_deg": [[100.0, 400.0, 700.0, 900.0] for _ in range(4)],
        }

    def real_m4_bad(self):
        model = self.good()
        model["distance_samples_mm"][3] = [19.84269692481342, 126.74775193765765, 137.54973144739324, 185.98706573643773]
        model["position_samples_deg"][3] = [-370.45, 153.15, 153.30, 410.0]
        model["slope"][3] = 4.671155683308748
        return model

    def test_reasonable_five_degrees_per_mm_is_not_rejected_by_nominal_radius(self):
        model = self.good()
        self.assertFalse(self.ns["rope_curve_quality_error"](model, 0))
        self.assertIsNotNone(self.ns["calibrated_rope_curve"](model, 0))
        self.assertTrue(self.ns["internal_model_readiness"](model)[0])

    def test_negative_encoder_direction_is_equally_valid(self):
        model = self.good()
        model["position_samples_deg"][0] = [-v for v in model["position_samples_deg"][0]]
        model["slope"][0] = -5.0
        self.assertFalse(self.ns["rope_curve_quality_error"](model, 0))
        self.assertIsNotNone(self.ns["calibrated_rope_curve"](model, 0))

    def test_four_metre_equivalent_radius_is_rejected_even_with_high_global_r2(self):
        model = self.real_m4_bad()
        reason = self.ns["rope_curve_quality_error"](model, 3)
        self.assertTrue(reason)
        self.assertIn("M4", reason)
        self.assertIn("20", reason)
        self.assertIsNone(self.ns["calibrated_rope_curve"](model, 3))
        ready, readiness_reason = self.ns["internal_model_readiness"](model)
        self.assertFalse(ready)
        self.assertIn("M4", readiness_reason)

    def test_no_forward_affine_fallback_after_curve_rejection(self):
        with self.assertRaises(ValueError):
            self.ns["_nominal_motor_degrees"](3, 22.0, 29.6, self.real_m4_bad())

    def test_no_inverse_affine_fallback_after_curve_rejection(self):
        self.assertIsNone(self.ns["encoder_positions_to_lengths"]([0.0] * 4, self.real_m4_bad()))

    def test_missing_source_in_current_cad_schema_is_not_live_ready(self):
        model = self.good()
        del model["calibration_source"]
        ready, reason = self.ns["internal_model_readiness"](model)
        self.assertFalse(ready)
        self.assertTrue(reason)

    def test_legacy_bare_curve_fixture_remains_a_math_utility_not_a_ready_model(self):
        model = {"distance_samples_mm": [[1.0, 2.0, 3.0]],
                 "position_samples_deg": [[1.0, 1.01, 1.02]]}
        self.assertIsNotNone(self.ns["calibrated_rope_curve"](model, 0))
        self.assertFalse(self.ns["internal_model_readiness"](model)[0])

    def test_previous_geometry_does_not_bypass_live_version_gate(self):
        model = self.good()
        model["geometry_version"] -= 1
        self.assertFalse(self.ns["internal_model_readiness"](model)[0])

    def test_nonfinite_sample_is_rejected_and_raw_data_not_modified(self):
        for bad in [math.nan, math.inf]:
            model = self.good()
            model["position_samples_deg"][0][1] = bad
            self.assertIsNone(self.ns["calibrated_rope_curve"](model, 0))
            self.assertFalse(self.ns["internal_model_readiness"](model)[0])
        model = self.real_m4_bad()
        before = copy.deepcopy(model)
        self.ns["rope_curve_quality_error"](model, 3)
        self.assertEqual(model, before)


if __name__ == "__main__":
    unittest.main()
