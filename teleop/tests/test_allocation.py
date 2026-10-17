#!/usr/bin/env python3
"""
Unit tests for teleop/allocation.py (plans/plan_teleop.md).

Pure numpy, no ROS/Docker dependency - runs in any Python env with numpy
installed (`python3 -m pytest teleop/tests/test_allocation.py` or
`python3 -m unittest` from user_repo_new/teleop/tests).
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from allocation import ThrustAllocator, _eul_to_rotm, _thruster_kt  # noqa: E402


# Real single-thruster + rudder geometry from
# ros2_ws/src/mavsim/debug/matsya_test.yaml, used throughout so these tests
# exercise the allocator against an actual shipped vessel config, not just
# synthetic numbers.
MATSYA_THRUSTER = {
    'actuator_id': 1,
    'thruster_location': [-1.4, 0.0, 0.1],
    'thruster_orientation': [0.0, 0.0, 0.0],
    'D_prop': 0.1,
    'J_vs_KT': [[-1, 0, 0, 1], [0, -1, 1, 0]],
    'n_max': 44.47,
    'n_min': -44.47,
}

MATSYA_RUDDER = {
    'actuator_id': 2,
    'control_surface_location': [-1.5, 0.0, 0.0],
    'control_surface_orientation': [0.0, 0.0, 0.0],
    'control_surface_delta_max': 35.0,
    'control_surface_delta_min': -35.0,
}


class TestEulToRotm(unittest.TestCase):
    """Pin _eul_to_rotm against hardcoded reference matrices so this copy
    can't silently drift from module_kinematics.py's eul_to_rotm(order='ZYX')."""

    def test_zero_angles_identity(self):
        np.testing.assert_allclose(_eul_to_rotm([0, 0, 0]), np.eye(3), atol=1e-12)

    def test_yaw_90(self):
        # Pure yaw +90deg: body x-axis -> world/local y-axis.
        r = _eul_to_rotm([0, 0, 90])
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-9)

    def test_roll_90(self):
        # Pure roll +90deg: body y-axis -> z-axis.
        r = _eul_to_rotm([90, 0, 0])
        np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, 1], atol=1e-9)

    def test_pitch_90(self):
        # Pure pitch +90deg: body x-axis -> -z-axis.
        r = _eul_to_rotm([0, 90, 0])
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 0, -1], atol=1e-9)

    def test_orthonormal(self):
        r = _eul_to_rotm([12.0, -34.0, 56.0])
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-9)


class TestSingleThrusterVessel(unittest.TestCase):
    """The matsya_test.yaml single-thruster, surge-aligned vessel."""

    def setUp(self):
        self.allocator = ThrustAllocator([MATSYA_THRUSTER], [])

    def test_dof_available_only_where_geometry_reaches(self):
        # A single on-axis thruster only has direct authority over surge (X);
        # its off-center location also gives it heave/pitch/yaw moment arms,
        # but sway (Y) and roll (K) should be exactly zero for this geometry
        # (orientation=[0,0,0], location has y=0).
        self.assertTrue(self.allocator.dof_available[0])  # X (surge)
        self.assertFalse(self.allocator.dof_available[1])  # Y (sway)
        self.assertFalse(self.allocator.dof_available[3])  # K (roll)

    def test_forward_and_reverse_are_opposite_sign(self):
        fwd = self.allocator.solve([1, 0, 0, 0, 0, 0])
        rev = self.allocator.solve([-1, 0, 0, 0, 0, 0])
        self.assertGreater(fwd['th_01'], 0)
        self.assertLess(rev['th_01'], 0)
        # Plausible magnitude: within the thruster's own RPM range.
        self.assertLessEqual(fwd['th_01'], MATSYA_THRUSTER['n_max'] + 1e-6)
        self.assertGreaterEqual(rev['th_01'], MATSYA_THRUSTER['n_min'] - 1e-6)

    def test_zero_command_round_trip(self):
        result = self.allocator.solve([0, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(result['th_01'], 0.0, places=6)


class TestControlSurfaceDominantAxis(unittest.TestCase):
    def test_stern_rudder_assigned_to_yaw(self):
        allocator = ThrustAllocator([], [MATSYA_RUDDER])
        meta = allocator._cs_meta['cs_02']
        self.assertEqual(meta['dof_idx'], 5)  # DOF_ORDER index of 'N' (yaw)

    def test_zero_command_round_trip(self):
        allocator = ThrustAllocator([], [MATSYA_RUDDER])
        result = allocator.solve([0, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(result['cs_02'], 0.0, places=6)


class TestMultiThrusterVessel(unittest.TestCase):
    """A hand-built symmetric 4-thruster layout (2 fore, 2 aft, each
    surge-aligned but offset in y) - gives yaw authority via differential
    thrust, unlike the single-thruster case above."""

    THRUSTERS = [
        {'actuator_id': 1, 'thruster_location': [1.0, 0.5, 0.0],
         'thruster_orientation': [0, 0, 0], 'D_prop': 0.15,
         'J_vs_KT': [[-1, 0, 0, 1], [0, -1, 1, 0]], 'n_max': 2000, 'n_min': -2000},
        {'actuator_id': 2, 'thruster_location': [1.0, -0.5, 0.0],
         'thruster_orientation': [0, 0, 0], 'D_prop': 0.15,
         'J_vs_KT': [[-1, 0, 0, 1], [0, -1, 1, 0]], 'n_max': 2000, 'n_min': -2000},
        {'actuator_id': 3, 'thruster_location': [-1.0, 0.5, 0.0],
         'thruster_orientation': [0, 0, 0], 'D_prop': 0.15,
         'J_vs_KT': [[-1, 0, 0, 1], [0, -1, 1, 0]], 'n_max': 2000, 'n_min': -2000},
        {'actuator_id': 4, 'thruster_location': [-1.0, -0.5, 0.0],
         'thruster_orientation': [0, 0, 0], 'D_prop': 0.15,
         'J_vs_KT': [[-1, 0, 0, 1], [0, -1, 1, 0]], 'n_max': 2000, 'n_min': -2000},
    ]

    def setUp(self):
        self.allocator = ThrustAllocator(self.THRUSTERS, [])

    def test_pseudo_inverse_reconstructs_reachable_wrench(self):
        b, b_pinv = self.allocator.B, self.allocator.B_pinv
        # A wrench actually in B's column space should round-trip through
        # B @ pinv(B) @ wrench.
        wrench = b @ np.array([10.0, -5.0, 3.0, 8.0])
        reconstructed = b @ (b_pinv @ wrench)
        np.testing.assert_allclose(reconstructed, wrench, atol=1e-6)

    def test_yaw_available_via_differential_thrust(self):
        self.assertTrue(self.allocator.dof_available[5])  # N (yaw)

    def test_max_wrench_scales_with_n_max(self):
        weaker = ThrustAllocator(
            [dict(t, n_max=200, n_min=-200) for t in self.THRUSTERS], []
        )
        self.assertTrue(np.all(weaker.max_wrench <= self.allocator.max_wrench + 1e-9))


class TestEmptyAllocator(unittest.TestCase):
    def test_no_actuators_returns_empty_dict(self):
        allocator = ThrustAllocator([], [])
        self.assertEqual(allocator.solve([0, 0, 0, 0, 0, 0]), {})
        np.testing.assert_array_equal(allocator.dof_available, np.zeros(6, dtype=bool))


if __name__ == '__main__':
    unittest.main()


class TestThrusterKT(unittest.TestCase):
    """Bollard K_T ahead/astern, following the simulator's ahead + reverse
    curves (ros2_ws/src/mavsim/mavsim/thruster_kt.py, 2026-10-17)."""

    AHEAD = [[-1, 0, 1], [0.3, 0.15, 0.0]]

    def test_old_single_curve_table_reads_as_before(self):
        self.assertEqual(_thruster_kt(MATSYA_THRUSTER), (1.0, -1.0))

    def test_mirror_is_the_default(self):
        self.assertEqual(_thruster_kt({'J_vs_KT': self.AHEAD}), (0.15, -0.15))

    def test_scaled(self):
        fwd, rev = _thruster_kt({'J_vs_KT': self.AHEAD, 'reverse_KT_mode': 'scaled', 'reverse_KT_percent': 50})
        self.assertAlmostEqual(fwd, 0.15)
        self.assertAlmostEqual(rev, -0.075)

    def test_custom_reverse_table(self):
        fwd, rev = _thruster_kt({'J_vs_KT': self.AHEAD, 'reverse_KT_mode': 'table',
                                 'J_vs_KT_reverse': [[-1, 0, 1], [-0.2, -0.1, 0.0]]})
        self.assertAlmostEqual(fwd, 0.15)
        self.assertAlmostEqual(rev, -0.1)

    def test_weaker_reverse_needs_more_rpm_and_saturates(self):
        # On the centreline, so the whole surge wrench goes to the one thruster.
        th = dict(MATSYA_THRUSTER, thruster_location=[-1.4, 0.0, 0.0], J_vs_KT=self.AHEAD,
                  reverse_KT_mode='scaled', reverse_KT_percent=50)
        allocator = ThrustAllocator([th], [])
        self.assertAlmostEqual(allocator.solve([1, 0, 0, 0, 0, 0])['th_01'], th['n_max'], places=4)
        self.assertAlmostEqual(allocator.solve([-1, 0, 0, 0, 0, 0])['th_01'], th['n_min'], places=4)
        half = allocator.solve([-0.25, 0, 0, 0, 0, 0])['th_01']
        mirror = ThrustAllocator([dict(th, reverse_KT_mode='mirror')], []).solve([-0.25, 0, 0, 0, 0, 0])['th_01']
        self.assertAlmostEqual(half, mirror * np.sqrt(2), places=4)    # half the K_T -> sqrt(2) x the RPM
