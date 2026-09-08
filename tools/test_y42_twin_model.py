import unittest
import numpy as np
from y42_twin_model import geometry,refine,simulate,plan

class TwinTests(unittest.TestCase):
    def test_jacobian_matches_finite_differences(self):
        q=np.array([43.,81.,28.,.13]);length,j,*_=geometry(q)
        for i in range(4):
            h=1e-5;a=q.copy();b=q.copy();a[i]+=h;b[i]-=h
            np.testing.assert_allclose(j[:,i],(geometry(a)[0]-geometry(b)[0])/(2*h),atol=1e-6)
    def test_refinement_preserves_corner_and_length(self):
        p=np.array([[20,20,25],[20,30,25],[30,30,25]],float);r=refine(p,.23)
        self.assertTrue(any(np.array_equal(row,p[1]) for row in r))
        self.assertLessEqual(np.linalg.norm(np.diff(r,axis=0),axis=1).max(),.230001)
        self.assertAlmostEqual(np.linalg.norm(np.diff(r,axis=0),axis=1).sum(),20)
    def test_sharp_corner_stops(self):
        p,t,v=plan([[20,20,25],[20,30,25],[30,30,25]],10,.5,80)
        idx=np.flatnonzero(np.all(p==[20,30,25],axis=1))[0];self.assertEqual(v[idx],0)
    def test_gravity_floor_and_nonnegative_tension(self):
        r=simulate({'points':[[50,74,25],[55,74,25]],'parameters':{'height':5}})
        self.assertTrue(all(f['pose'][2]>=25 for f in r['frames']))
        self.assertTrue(all(min(f['tension'])>=0 for f in r['frames']))
        self.assertAlmostEqual(r['frames'][-1]['pose'][2],25)
        self.assertEqual(r['frames'][-1]['target'][2],30)
    def test_spacing_does_not_change_curve(self):
        a=simulate({'shape':'square','parameters':{'spacing':.25,'speed':50}})
        b=simulate({'shape':'square','parameters':{'spacing':.5,'speed':50}})
        self.assertGreater(a['point_count'],b['point_count'])
        self.assertLess(abs(a['duration']-b['duration']),.03)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):simulate({'parameters':{'mass':float('nan')}})
    def test_actuator_updates_on_configured_clock_and_preserves_corner(self):
        points=[[50,70,25],[54,70,25],[54,74,25]]
        r=simulate({'points':points,'parameters':{'control_hz':25}})
        commands=r['commands']
        np.testing.assert_allclose(np.diff([c['t'] for c in commands]),.04,atol=1e-10)
        for point in points:
            self.assertTrue(any(np.allclose(c['target'],point,atol=1e-10,rtol=0) for c in commands))
        quantum=12*np.pi/180*.1
        lengths=np.asarray([c['rest_lengths_mm'] for c in commands])
        units=(lengths-lengths[0])/quantum
        np.testing.assert_allclose(units,np.round(units),atol=1e-9)
        for frame in r['frames']:
            latest=max((c for c in commands if c['t']<=frame['t']+1e-9),key=lambda c:c['t'])
            np.testing.assert_allclose(frame['command_target'],latest['target'])
    def test_circle_timing_not_artificially_slowed_by_refinement(self):
        from y42_twin_model import preset
        _,t1,_=plan(preset('circle'),50,.25,80)
        _,t2,_=plan(preset('circle'),50,.5,80)
        self.assertLess(abs(t1[-1]-t2[-1]),.01)

if __name__=='__main__':unittest.main()
