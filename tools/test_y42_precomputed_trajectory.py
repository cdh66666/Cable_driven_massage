import copy
import tempfile
import unittest
from unittest.mock import patch
from test_y42_calibration_flow import isolated_web_module

class PrecomputeTests(unittest.TestCase):
    def test_encoded_speed_covers_rounded_position_increment(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            c=w.calibration_data()
            for a,b in [(0.049,0.051),(-0.049,-0.051),(0.151,0.349)]:
                with patch.object(w,'grid_motor_positions_at',side_effect=[[a]*4,[b]*4]):
                    packet=w.trajectory_frame(0,0,1,1,.01,[1500]*4,c)
                for i in range(4):
                    encoded_rpm=int.from_bytes(packet[7+14*i:9+14*i],'big')/10
                    delta=abs(round(b*10)-round(a*10))/10
                    self.assertGreaterEqual(encoded_rpm*6*.01+1e-9,delta)

    def test_frames_equal_previous_kinematics_and_exact_last_point(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            self.assertAlmostEqual(w.next_trajectory_deadline(0,.0001,.0003,.01),.01)
            self.assertAlmostEqual(w.next_trajectory_deadline(0,0,.015,.01),.015)
            self.assertAlmostEqual(w.next_trajectory_deadline(0,.016,.0162,.01),.026)
            w.STOP_EVENT.clear()
            c=w.calibration_data()
            c.update(calibration_source='brute_force_four_corner_torque',slope=[-4.8,4.7,-4.9,4.6],intercept=[100,200,300,400])
            spec={'duration_ms':105,'path':[(10.,20.),(12.,25.)],'cumulative':[0.,1.],'path_absolute':True}
            frozen=copy.deepcopy(spec);offset=[1.,-2.,3.,-4.]
            plan=w.precompute_position_trajectory('line',spec,c,offset,[1500]*4,.01)
            self.assertEqual(len(plan),11)
            self.assertEqual(plan[-1][1],(12.,25.))
            for i,item in enumerate(plan):
                t=i*.01; nt=min(.105,t+.01)
                x,y=w.sample_path('line',t/.105,spec); nx,ny=w.sample_path('line',nt/.105,spec)
                target=[v+offset[j] for j,v in enumerate(w.grid_motor_positions_at(nx,ny,c))]
                expected=w.trajectory_frame(x,y,nx,ny,max(.001,nt-t),[1500]*4,c,target_overrides=target)
                self.assertEqual(item[0],expected)
                self.assertEqual(len(item[0]),61)
            self.assertEqual(spec,frozen)
            # Frozen packets can be replayed even when every geometry API is unavailable.
            with patch.object(w,'sample_path',side_effect=AssertionError('geometry during playback')),patch.object(w,'trajectory_frame',side_effect=AssertionError('packing during playback')):
                packets=[item[0] for item in plan]
                self.assertEqual(len(packets),11)
            w.STOP_EVENT.set()
            with self.assertRaisesRegex(RuntimeError,'停止轨迹预计算'):
                w.precompute_position_trajectory('line',spec,c,offset,[1500]*4,.01)

if __name__=='__main__':unittest.main()
