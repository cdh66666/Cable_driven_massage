import copy
import tempfile
import unittest
from test_y42_calibration_flow import isolated_web_module

class SessionYawTrialTests(unittest.TestCase):
    def test_start_is_unchanged_and_translation_uses_same_rigid_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            c=w.calibration_data()
            c.update(calibration_source='brute_force_four_corner_torque',
                     slope=[-4.8,4.8,-4.8,4.8],intercept=[10,20,30,40])
            baseline=copy.deepcopy(c)
            sx,sy=70.,74.; yaw=38.
            start=[w.motor_degrees(i,sx,sy,c) for i in range(4)]
            c['_trial_fixed_yaw_datum']={'yaw_deg':yaw,'start_xy':[sx,sy]}
            self.assertEqual(start,[w.motor_degrees(i,sx,sy,c) for i in range(4)])
            initial=w.planar_platform_cable_lengths(sx,sy,yaw,c)
            for x,y in [(55,59),(40,74),(55,89),(70,74)]:
                lengths=w.planar_platform_cable_lengths(x,y,yaw,c)
                for i in range(4):
                    expected=start[i]+w.effective_slopes(c)[i]*(lengths[i]-initial[i])
                    self.assertAlmostEqual(w.motor_degrees(i,x,y,c),expected,places=8)
            self.assertNotIn('_trial_fixed_yaw_datum',baseline)

if __name__=='__main__':unittest.main()
