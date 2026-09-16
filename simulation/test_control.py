"""Tests include physical outcomes, independent of the controller's internals."""
import unittest
import numpy as np
from .control import allocate_forces, reference
from .model import make_model
from .run import experiment


class StanceTests(unittest.TestCase):
    def test_robot_is_unanchored_and_actuated(self):
        model, data = make_model(payload_kg=3.)
        self.assertEqual((model.nq, model.nv, model.nu), (19, 18, 12))
        self.assertEqual(model.neq, 0)
        self.assertAlmostEqual(model.body_mass[model.body("payload").id], 3.)
        self.assertTrue(np.isfinite(data.qpos).all())

    def test_force_allocator_respects_friction_under_infeasible_request(self):
        feet = np.array([[.25,.25,0],[.25,-.25,0],[-.25,.25,0],[-.25,-.25,0]])
        forces, ok = allocate_forces(feet, np.array([0,0,.4]), np.array([300,100,150,0,0,0]), np.tile([0,0,37.5],4), mu=.4)
        self.assertTrue(ok)
        f = forces.reshape(4,3)
        self.assertTrue(np.all(f[:,2] >= -1e-6))
        self.assertTrue(np.all(np.abs(f[:,0])+np.abs(f[:,1]) <= .4*f[:,2]+1e-4))

    def test_static_force_balance(self):
        feet = np.array([[.25,.25,0],[.25,-.25,0],[-.25,.25,0],[-.25,-.25,0]])
        f, ok = allocate_forces(feet, np.array([0,0,.4]), np.array([0,0,160,0,0,0]), np.tile([0,0,40.],4))
        self.assertTrue(ok)
        np.testing.assert_allclose(f.reshape(4,3).sum(axis=0), [0,0,160], atol=.1)

    def test_height_reference_is_continuous(self):
        for boundary in (3.,5.,8.,10.):
            a, b = reference(boundary-1e-6), reference(boundary+1e-6)
            self.assertAlmostEqual(a[0], b[0], places=5)
            self.assertAlmostEqual(a[1], b[1], places=5)

    def test_nominal_physics_and_baseline(self):
        active = experiment()
        baseline = experiment("joint_pd")
        a, b = active["metrics"], baseline["metrics"]
        self.assertFalse(a["fell"])
        self.assertEqual(a["solverFailures"], 0)
        self.assertLess(a["peakTiltDeg"], 3.)
        self.assertLess(a["heightRmseMm"], 6.)
        self.assertLess(a["rmsTiltDeg"], b["rmsTiltDeg"]*.6)
        self.assertLess(a["heightRmseMm"], b["heightRmseMm"]*.6)
        self.assertTrue(all(t is not None and t < 1. for t in a["recoverySeconds"]))
        for frame in active["frames"]:
            self.assertLessEqual(frame["torque"], 32.)
            self.assertTrue(np.isfinite(frame["qpos"]).all())


if __name__ == "__main__":
    unittest.main()
