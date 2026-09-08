import math,tempfile,unittest
from test_y42_calibration_flow import isolated_web_module

class CornerAccelerationTests(unittest.TestCase):
    def test_handdraw_workspace_coordinates_are_not_scaled_again(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            actual=w.handdraw_point_to_workspace(.2,.3,{},'workspace_normalized')
            for a,b in zip(actual,(22,103.6)):self.assertAlmostEqual(a,b)
            legacy=w.handdraw_point_to_workspace(.2,.3,{},'inset_normalized')
            for a,b in zip(legacy,(28.6,97.68)):self.assertAlmostEqual(a,b)
            with self.assertRaises(ValueError):
                w.handdraw_point_to_workspace(.05,.5,{},'workspace_normalized')
            for x,y in ((.1,.1),(.9,.9)):
                actual=w.handdraw_point_to_workspace(x,y,{},'workspace_normalized')
                self.assertAlmostEqual(actual[0]/110,x)
                self.assertAlmostEqual(1-actual[1]/148,y)

    def test_inset_does_not_change_calibration_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            self.assertEqual(w.drawable_workspace(),(0,0,110,148))
            left,bottom,width,height=w.drawing_target_bounds()
            for actual,expected in zip((left,bottom,width,height),(11,14.8,88,118.4)):
                self.assertAlmostEqual(actual,expected)
            shape=w.test_shape_spec('rectangle',110,148,speed_mm_s=50)
            self.assertAlmostEqual(shape['origin_x'],left)
            self.assertAlmostEqual(shape['origin_y'],bottom)
            self.assertAlmostEqual(shape['draw_width'],width)
            self.assertAlmostEqual(shape['draw_height'],height)

    def test_complete_seven_pattern_plan_has_no_vector_acceleration_spikes(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            spec=w.all_test_pattern_spec(110,148,speed_mm_s=90)
            duration=spec['duration_ms']/1000
            samples=[w.sample_path('custom',min(1,i*.01/duration),spec) for i in range(math.ceil(duration/.01)+1)]
            velocities=[((b[0]-a[0])/.01,(b[1]-a[1])/.01) for a,b in zip(samples,samples[1:])]
            self.assertLessEqual(max(math.dist(a,b)/.01 for a,b in zip(velocities,velocities[1:])),450.01)
            self.assertEqual(len(spec['sequence_shapes']),6)
            self.assertLess(duration,45)

    def test_test_patterns_follow_saved_drawing_speed(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            w.calibration_data=lambda: {'path_speed_mm_s':50}
            slow=w.all_test_pattern_spec(110,148)
            fast=w.all_test_pattern_spec(110,148,speed_mm_s=90)
            self.assertEqual(slow['requested_speed_mm_s'],50)
            self.assertGreater(slow['duration_ms'],fast['duration_ms'])
            for kind in w.ALL_TEST_PATTERN_KINDS:
                self.assertEqual(w.test_shape_spec(kind,110,148),w.test_shape_spec(kind,110,148,speed_mm_s=50))
            duration=slow['duration_ms']/1000
            samples=[w.sample_path('custom',min(1,i*.01/duration),slow) for i in range(math.ceil(duration/.01)+1)]
            self.assertLessEqual(max(math.dist(a,b)/.01 for a,b in zip(samples,samples[1:])),50.01)

    def test_sharp_corners_obey_vector_acceleration_without_subdivision_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            w=isolated_web_module(directory)
            path=w.resample_polyline([(11,133.2),(99,133.2),(99,14.8),(11,14.8),(11,133.2)])
            original=list(path)
            times,speeds=w.time_parameterized_path(path,50,450,start_speed_mm_s=0,end_speed_mm_s=0)
            for corner in [(99,133.2),(99,14.8),(11,14.8)]:self.assertEqual(speeds[path.index(corner)],0)
            self.assertEqual(path,original)
            self.assertEqual(sum(v==0 for v in speeds),5)
            samples=[w.sample_cumulative_path(path,times,min(1,i*.01/times[-1]),speeds) for i in range(math.ceil(times[-1]/.01)+1)]
            velocities=[((b[0]-a[0])/.01,(b[1]-a[1])/.01) for a,b in zip(samples,samples[1:])]
            peak=max(math.dist(a,b)/.01 for a,b in zip(velocities,velocities[1:]))
            self.assertLessEqual(peak,450.01)
            self.assertLess(times[-1],10)

if __name__=='__main__':unittest.main()
