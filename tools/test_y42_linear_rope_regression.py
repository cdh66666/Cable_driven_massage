"""No serial access: the measured M4 plateau must not flatten commanded payout."""
import copy
import math
import tempfile
import unittest
from test_y42_calibration_flow import isolated_web_module

class LinearRopeRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.storage=tempfile.TemporaryDirectory(prefix='y42-linear-regression-')
        cls.web=isolated_web_module(cls.storage.name)
    @classmethod
    def tearDownClass(cls):
        cls.storage.cleanup()
    def test_live_plateau_uses_independent_gain_without_mutating_samples(self):
        w=self.web
        data=w.calibration_data()
        data.update(calibration_source='brute_force_four_corner_torque',slope=[-4.8242145,4.8573469,-4.8531302,4.6528746],intercept=[712.52,-535.32,284.38,-1541.10],slope_trim_percent=[0.0]*4,
                    distance_samples_mm=[[19.8427,126.74775,137.54973,185.98707] for _ in range(4)],
                    position_samples_deg=[[617.3,89.4,61.5,-186.1],[-437.4,92.6,112.55,374.55],[193.15,-349,-378.2,-610],[-1446.95,-932.1,-930.9,-667]])
        before=copy.deepcopy(data)
        self.assertIsNone(w.calibrated_rope_curve(data,3))
        self.assertIsNone(w._corner_position_residuals(60,70,data))
        ax,ay=w.workspace_anchor_coordinates(data)
        for x,y in [(4,4),(55,74),(106,144),(106,100)]:
            for i in range(4):
                expected=data['intercept'][i]+w.effective_slopes(data)[i]*max(0,math.hypot(x-ax[i],y-ay[i])-w.TRAJECTORY_PRETENSION_MM)
                self.assertAlmostEqual(w.motor_degrees(i,x,y,data),expected,places=8)
        self.assertEqual(data,before)
        self.assertGreater((137.54973-126.74775)*data['slope'][3],50)
        data.update(valid=True,workspace_calibration_verified=True,internal_model_verified=True,geometry_version=w.GEOMETRY_VERSION,r2=[.99]*4)
        self.assertTrue(w.persisted_calibration_structure_ready(data)[0])
        data['slope'][3]=0
        self.assertFalse(w.persisted_calibration_structure_ready(data)[0])

if __name__=='__main__':unittest.main()
