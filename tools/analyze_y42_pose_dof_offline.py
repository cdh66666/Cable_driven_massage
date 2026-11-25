#!/usr/bin/env python3
"""Local cable-observation rank with free ball roll/pitch (offline only).

Uses nominal cylinder tangents and explicitly assumed exit heights.  This is a
structural observability illustration, not an estimated physical ball pose.
"""
import json
import math

import numpy as np

from analyze_y42_start_pose_offline import AXES, ATTACHMENTS
from y42_spool_geometry import cylindrical_payout


def pose_observation(x, y, roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    offsets = np.column_stack((ATTACHMENTS, np.zeros(4))) @ (rz @ ry @ rx).T
    endpoints = np.array([x, y, 25]) + offsets
    references = [-math.pi / 4, -3 * math.pi / 4, 3 * math.pi / 4, math.pi / 4]
    branches = [1, -1, 1, -1]
    jacobian = []
    for motor in range(4):
        tangent = cylindrical_payout(AXES[motor], endpoints[motor, :2], 12, branches[motor], reference_phi_rad=references[motor])["tangent_xy_mm"]
        # Fixed 10 mm exit height is only an illustration; allowing unknown
        # per-axis heights/layers adds latent variables, never removes them.
        outward = endpoints[motor] - np.array([*tangent, 10])
        outward /= np.linalg.norm(outward)
        rotation_derivatives = np.cross(offsets[motor], outward) / 20
        jacobian.append([outward[0], outward[1], *rotation_derivatives])
    jacobian = np.array(jacobian)
    _, singular, vt = np.linalg.svd(jacobian, full_matrices=True)
    hidden = vt[-1]
    return {
        "assumed_pose_xy_roll_pitch_yaw_deg": [x, y, roll_deg, pitch_deg, yaw_deg],
        "jacobian_shape": list(jacobian.shape),
        "rank": int(np.linalg.matrix_rank(jacobian)),
        "singular_values": singular.tolist(),
        "unobservable_local_direction_xy_and20mm_scaled_rotation": hidden.tolist(),
        "hidden_direction_residual_norm": float(np.linalg.norm(jacobian @ hidden)),
        "local_closure_dimensions_after_free_pose": int(4 - np.linalg.matrix_rank(jacobian)),
    }


if __name__ == "__main__":
    print(json.dumps([pose_observation(*pose) for pose in [(55, 74, 0, 0, 0), (22, 29.6, 0, 0, 0), (22, 29.6, 5, 3, 0)]], indent=2))
