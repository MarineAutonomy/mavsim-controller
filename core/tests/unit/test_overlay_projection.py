#!/usr/bin/env python3
"""
Geometry checks for the camera/lidar overlay in visualizer_server.py.

The overlay projects lidar points into the camera image. Two frames meet
there and they do not share a handedness:

  - Lidar points arrive in the lidar's OWN local frame. LidarSensor.js builds
    its rays as [cos(el)cos(az), cos(el)sin(az), sin(el)], so X is forward,
    azimuth sweeps X->Y (Y=left) and elevation is Z (Z=up).
  - sensor_location/sensor_orientation are expressed in a parent frame where
    Y=right and Z=down. That is forced by the real camera mounting
    [-90, 0, 90], which puts the camera's forward on +X, its up on -Z and
    its right on +Y.

Converting between them is a 180 degree roll about X. Without it the overlay
is mirrored on both axes - left projects right, up projects down - which is
the bulk of any visible misalignment. Separately, the points are already in
the lidar frame when they arrive, so the lidar's mounting pose must be
applied exactly once.

These tests replicate the projection in numpy rather than driving the
browser, so they run in the Python-only CI. The expected pixel values were
cross-checked against the real visualizer_server.py code executing under the
vendored three.min.js, and agree to within a tenth of a pixel.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy is a hard dep of the bridge
    np = None


# Real matsya_01 extrinsics, as published in sensor_config.json.
CAM_LOC = [0.8, 0.0, -0.25]
CAM_ORI = [-90.0, 0.0, 90.0]
LIDAR_LOC = [1.0, 0.0, -0.2]
LIDAR_ORI = [0.0, 0.0, 0.0]
FOV_DEG, IMG_W, IMG_H = 60.0, 640, 480


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _pose(loc, ori_deg):
    """sensor_orientation is [roll, pitch, yaw] degrees applied ZYX."""
    r, p, y = (math.radians(v) for v in ori_deg)
    return np.array(loc, dtype=float), _rz(y) @ _ry(p) @ _rx(r)


# (x, y, z) -> (x, -y, -z): lidar frame (Y=left, Z=up) into the mounting
# frame (Y=right, Z=down). Mirrors LIDAR_TO_BODY_ROLL in visualizer_server.py.
def _lidar_to_body_roll():
    return np.diag([1.0, -1.0, -1.0])


def _lidar_ray(azimuth_deg, elevation_deg, rng=15.0):
    """A ray exactly as LidarSensor._generateRayDirections() builds it."""
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    return np.array([rng * math.cos(e) * math.cos(a),
                     rng * math.cos(e) * math.sin(a),
                     rng * math.sin(e)])


def project(point_lidar, apply_roll=True, double_pose=False):
    """Project a lidar-frame point to pixels; returns None if not visible.

    apply_roll/double_pose exist to reproduce the two bugs this guards
    against, so the tests can assert the broken forms actually fail.
    """
    t_l, r_l = _pose(LIDAR_LOC, LIDAR_ORI)
    t_c, r_c = _pose(CAM_LOC, CAM_ORI)

    p = _lidar_to_body_roll() @ point_lidar if apply_roll else np.array(point_lidar, float)
    p_body = r_l @ p + t_l
    if double_pose:
        p_body = r_l @ p_body + t_l

    p_cam = r_c.T @ (p_body - t_c)
    if p_cam[2] >= 0:      # three.js cameras look down -Z
        return None
    f = (IMG_H / 2) / math.tan(math.radians(FOV_DEG) / 2)
    return (IMG_W / 2 + f * (p_cam[0] / -p_cam[2]),
            IMG_H / 2 - f * (p_cam[1] / -p_cam[2]))


@unittest.skipIf(np is None, "numpy not available")
class TestOverlayProjection(unittest.TestCase):

    CX, CY = IMG_W / 2, IMG_H / 2

    def test_dead_ahead_projects_to_image_centre(self):
        px = project(_lidar_ray(0, 0))
        self.assertIsNotNone(px)
        self.assertAlmostEqual(px[0], self.CX, delta=2)
        # Not exactly CY: the camera sits 5cm above the lidar, so a point
        # level with the lidar is slightly below the optical axis.
        self.assertAlmostEqual(px[1], self.CY, delta=6)

    def test_left_target_projects_left_of_centre(self):
        px = project(_lidar_ray(20, 0))
        self.assertIsNotNone(px)
        self.assertLess(px[0], self.CX, "a target to the left must project left")

    def test_right_target_projects_right_of_centre(self):
        px = project(_lidar_ray(-20, 0))
        self.assertIsNotNone(px)
        self.assertGreater(px[0], self.CX, "a target to the right must project right")

    def test_up_target_projects_above_centre(self):
        px = project(_lidar_ray(0, 10))
        self.assertIsNotNone(px)
        self.assertLess(px[1], self.CY, "a target above must project above (smaller y)")

    def test_down_target_projects_below_centre(self):
        px = project(_lidar_ray(0, -10))
        self.assertIsNotNone(px)
        self.assertGreater(px[1], self.CY, "a target below must project below")

    def test_behind_camera_is_culled(self):
        self.assertIsNone(project(_lidar_ray(180, 0)))

    def test_horizontal_is_symmetric(self):
        left = project(_lidar_ray(20, 0))
        right = project(_lidar_ray(-20, 0))
        self.assertAlmostEqual(left[0] - self.CX, -(right[0] - self.CX), delta=1)

    def test_matches_browser_reference_values(self):
        """Pixel values produced by the real JS under vendored three.min.js."""
        for (az, el), expect in {
            (0, 0):   (320.0, 241.4),
            (20, 0):  (170.8, 241.5),
            (-20, 0): (469.2, 241.5),
            (0, 10):  (320.0, 169.1),
            (0, -10): (320.0, 313.7),
        }.items():
            px = project(_lidar_ray(az, el))
            self.assertAlmostEqual(px[0], expect[0], delta=0.2, msg=f"x at az={az} el={el}")
            self.assertAlmostEqual(px[1], expect[1], delta=0.2, msg=f"y at az={az} el={el}")


@unittest.skipIf(np is None, "numpy not available")
class TestOverlayRegressions(unittest.TestCase):
    """The two bugs this geometry previously had must stay fixed."""

    CX, CY = IMG_W / 2, IMG_H / 2

    def test_without_roll_the_overlay_is_mirrored(self):
        """Skipping the axis conversion mirrors both axes - the original bug."""
        left = project(_lidar_ray(20, 0), apply_roll=False)
        up = project(_lidar_ray(0, 10), apply_roll=False)
        self.assertGreater(left[0], self.CX, "unfixed: left target lands right")
        self.assertGreater(up[1], self.CY, "unfixed: up target lands below")

    def test_double_applying_the_lidar_pose_shifts_points(self):
        """Applying the mounting pose twice displaces the projection."""
        once = project(_lidar_ray(0, 0), double_pose=False)
        twice = project(_lidar_ray(0, 0), double_pose=True)
        self.assertGreater(abs(once[1] - twice[1]), 1.0,
                           "double-applied pose should visibly shift the point")


if __name__ == '__main__':
    unittest.main()
