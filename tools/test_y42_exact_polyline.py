"""Exercise the actual controller conversion/resampling without hardware imports."""
import ast
import math
from pathlib import Path
import unittest
from y42_handwriting_plan_replay import conversion_environment


class ExactPolyline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env=conversion_environment(ast.parse(Path(__file__).with_name('y42_pc_direct_web.py').read_text(encoding='utf-8')))

    def assert_vertices_and_length(self, original, output):
        cursor=0
        for point in original:
            while cursor<len(output) and output[cursor]!=point: cursor+=1
            self.assertLess(cursor,len(output),f'lost vertex {point}')
            cursor+=1
        length=lambda p:sum(math.dist(a,b) for a,b in zip(p,p[1:]))
        self.assertAlmostEqual(length(original),length(output),places=10)

    def test_tiny_zigzags_closing_point_and_duplicates_survive(self):
        raw=[(0.,0.),(.01,.02),(.02,0.),(.02,0.),(.021,.001),(0.,0.)]
        output=self.env['resample_polyline'](raw,.05,100)
        self.assert_vertices_and_length(raw,output)

    def test_density_budget_reduces_insertions_not_original_vertices(self):
        raw=[(0.,0.),(3.123456789,0.),(3.123456789,4.123456789),(0.,4.123456789),(0.,0.)]
        output=self.env['resample_polyline'](raw,.05,11)
        self.assertEqual(len(output),11)
        self.assert_vertices_and_length(raw,output)

    def test_too_many_original_points_are_rejected_not_decimated(self):
        with self.assertRaises(ValueError):self.env['resample_polyline']([(i,0) for i in range(12)],.1,11)

    def test_production_input_conversion_preserves_close_points_and_exact_endpoint(self):
        raw=[{'x':.2,'y':.2},{'x':.20001,'y':.20002},{'x':.20002,'y':.2},{'x':.3,'y':.3},{'x':.30001,'y':.30001}]
        refined,original,count,_=self.env['convert']([raw],{})
        self.assertEqual(count,len(raw))
        self.assert_vertices_and_length(original[0],refined[0])

    def test_100hz_targets_visit_every_vertex_and_never_bridge_a_corner(self):
        raw=[(0.,0.),(10.,0.),(10.,10.),(10.01,10.02),(0.,0.)]
        phases,_,duration,_=self.env['plan_custom_motion_segments']([raw],50,150,exact_vertices=True)
        spec=dict(motion_segments=phases,duration_ms=round(duration*1000))
        sampled=[self.env['custom_motion_phase'](min(1,i*.01/duration),spec)['xy'] for i in range(round(duration/.01)+1)]
        self.assert_vertices_and_length(raw,sampled)
        for phase in phases:
            for t in phase['cumulative']:self.assertAlmostEqual(t/.01,round(t/.01),places=8)
            for a,b,t0,t1 in zip(phase['path'],phase['path'][1:],phase['cumulative'],phase['cumulative'][1:]):
                distance=math.dist(a,b);seconds=t1-t0
                self.assertLessEqual(2*distance/seconds,phase['cruise_speed_mm_s']+1e-8)
                self.assertLessEqual(4*distance/seconds**2,phase['acceleration_mm_s2']+1e-8)

    def test_25hz_plan_visits_original_vertices_on_its_actual_clock(self):
        raw=[(0.,0.),(4.,0.),(4.,3.),(4.01,3.02),(0.,0.)]
        phases,_,duration,_=self.env['plan_custom_motion_segments'](
            [raw],10,10,exact_vertices=True,control_tick_s=.04)
        spec=dict(motion_segments=phases,duration_ms=round(duration*1000))
        sampled=[self.env['custom_motion_phase'](min(1,i*.04/duration),spec)['xy']
                 for i in range(round(duration/.04)+1)]
        self.assert_vertices_and_length(raw,sampled)
        for phase in phases:
            for t in phase['cumulative']:
                self.assertAlmostEqual(t/.04,round(t/.04),places=8)

    def test_nonfinite_limits_are_rejected(self):
        from y42_exact_path import exact_vertex_profile
        for bad in (float('nan'),float('inf'),0,-1):
            with self.assertRaises(ValueError):
                exact_vertex_profile([(0,0),(1,1)],bad,10)



if __name__=='__main__':unittest.main()
