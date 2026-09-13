#!/usr/bin/env python3
"""
Geometry checks for the camera/lidar overlay in visualizer_server.py.

The overlay projects lidar points into the camera image. The lidar's point
frame and the mounting-pose frame are the SAME frame - plain NED, X=forward,
Y=right, Z=down - so no axis conversion is applied between them.

That was established from live data using only range and height, which no
frame convention can alter:

  - water returns (<2m) under a sensor mounted 0.2m up -> z = +0.20, so
    down is +z;
  - the receding cliff on the LEFT of the camera image -> 100% y < 0, and
    the close, tall cliff on the RIGHT -> 100% y > 0, so right is +y.

Two wrong conversions were shipped before this was settled: a 180-degree
roll, which inverted the vertical, and then a mirror in Y alone, which
swapped left and right. The mirror drew the close cliff's returns over the
far cliff and vice versa, so the points overshot the low cliff and fell
short of the tall one - an asymmetry that looked like a height error. It
came from misreading which cliff was which in the image, and every check
that "confirmed" it compared the projection against a prediction that had
the same assumption baked in. The tests below therefore classify points by
RANGE and check which half of the image they land in.

Separately, the points are already in the lidar frame when they arrive, so
the mounting pose must be applied exactly once.

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


# Identity: the lidar frame IS the NED mounting frame. Mirrors LIDAR_TO_BODY
# in visualizer_server.py.
def _lidar_to_body():
    return np.eye(3)


def _lidar_ray(azimuth_deg, elevation_deg, rng=15.0):
    """A point at this azimuth/elevation in the lidar's NED frame.

    X=forward, Y=RIGHT, Z=down. A positive azimuth is to the RIGHT and a
    positive elevation (above the sensor) is a NEGATIVE z.
    """
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    return np.array([rng * math.cos(e) * math.cos(a),
                     rng * math.cos(e) * math.sin(a),
                     -rng * math.sin(e)])


def project(point_lidar, mirror_y=False, flip_z_too=False, double_pose=False):
    """Project a lidar-frame point to pixels; returns None if not visible.

    mirror_y/flip_z_too/double_pose reproduce the bugs this guards against,
    so the tests can assert the broken forms actually fail.
    """
    t_l, r_l = _pose(LIDAR_LOC, LIDAR_ORI)
    t_c, r_c = _pose(CAM_LOC, CAM_ORI)

    m = _lidar_to_body()
    if mirror_y:
        m = np.diag([1.0, -1.0, 1.0])
    if flip_z_too:
        m = np.diag([1.0, -1.0, -1.0])
    p = m @ point_lidar
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

    def test_right_target_projects_right_of_centre(self):
        px = project(_lidar_ray(20, 0))          # +azimuth = right in NED
        self.assertIsNotNone(px)
        self.assertGreater(px[0], self.CX, "a target to the right must project right")

    def test_left_target_projects_left_of_centre(self):
        px = project(_lidar_ray(-20, 0))
        self.assertIsNotNone(px)
        self.assertLess(px[0], self.CX, "a target to the left must project left")

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
        right = project(_lidar_ray(20, 0))
        left = project(_lidar_ray(-20, 0))
        self.assertAlmostEqual(right[0] - self.CX, -(left[0] - self.CX), delta=1)

    def test_matches_browser_reference_values(self):
        """Pixel values produced by the real JS under vendored three.min.js."""
        for (az, el), expect in {
            (0, 0):   (320.0, 241.4),
            (20, 0):  (469.2, 241.5),   # +az = right
            (-20, 0): (170.8, 241.5),
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

    def test_mirroring_y_swaps_left_and_right(self):
        """The mirror-in-Y that was shipped: a right target lands left.

        On a real scene this drew the close cliff's returns over the far
        cliff and vice versa, which read as a height inconsistency.
        """
        right = project(_lidar_ray(20, 0), mirror_y=True)
        self.assertLess(right[0], self.CX, "mirrored: right target lands left")

    def test_flipping_z_as_well_inverts_the_vertical(self):
        """A 180-degree roll also flips Z, hanging cliffs below the horizon.

        This was shipped briefly and is what made the overlay look upside
        down against a real scene.
        """
        up = project(_lidar_ray(0, 10), flip_z_too=True)
        down = project(_lidar_ray(0, -10), flip_z_too=True)
        self.assertGreater(up[1], self.CY, "over-flipped: up target lands below")
        self.assertLess(down[1], self.CY, "over-flipped: down target lands above")

    def test_double_applying_the_lidar_pose_shifts_points(self):
        """Applying the mounting pose twice displaces the projection."""
        once = project(_lidar_ray(0, 0), double_pose=False)
        twice = project(_lidar_ray(0, 0), double_pose=True)
        self.assertGreater(abs(once[1] - twice[1]), 1.0,
                           "double-applied pose should visibly shift the point")


if __name__ == '__main__':
    unittest.main()


@unittest.skipIf(np is None, "numpy not available")
class TestPointCloudOrbit(unittest.TestCase):
    """Orbit controls for the Z-down point cloud viewer.

    The viewer renders the lidar frame directly (X=forward, Y=left, Z=down)
    with up = -Z. Flipping the up-vector also reverses screen-right, so BOTH
    drag axes need the opposite sign to the usual Z-up orbit - fixing only
    the vertical one leaves the horizontal drag inverted.
    """

    UP = np.array([0.0, 0.0, -1.0])

    def _eye(self, theta, phi, radius=6.0):
        """Mirrors _updateCamera(): phi measured from -Z."""
        return np.array([radius * math.sin(phi) * math.cos(theta),
                         radius * math.sin(phi) * math.sin(theta),
                         -radius * math.cos(phi)])

    def _screen_right(self, eye):
        fwd = -eye / np.linalg.norm(eye)          # looking at the origin
        right = np.cross(fwd, self.UP)
        return right / np.linalg.norm(right)

    def _screen_y(self, eye, feature):
        """Where a world point sits vertically on screen, as a depth-normalised
        offset: larger means higher up the image."""
        fwd = -eye / np.linalg.norm(eye)
        up = np.cross(self._screen_right(eye), fwd)
        d = feature - eye
        return float(d @ up) / float(d @ fwd)

    def test_default_view_is_above_the_scene(self):
        """phi < pi/2 must put the eye above, i.e. at negative z."""
        eye = self._eye(math.pi / 4, math.pi / 3)
        self.assertLess(eye[2], 0, "default eye is below the scene")

    def test_drag_right_swings_camera_left(self):
        """Grab-the-scene: drag right, the scene follows, the eye goes left."""
        theta, phi = math.pi / 4, math.pi / 3
        e0 = self._eye(theta, phi)
        e1 = self._eye(theta + 0.2, phi)          # theta += dx
        self.assertLess(float((e1 - e0) @ self._screen_right(e0)), 0,
                        "drag right should move the eye left around the target")

    def test_drag_down_moves_the_scene_down(self):
        """The test that matters is perceived motion, not eye height.

        Grab-the-scene means the scene follows the cursor, so a downward
        drag must push the view's content DOWN the image. That happens when
        the eye RISES (phi -= dy), which is the opposite of what "drag down
        = lower the camera" would suggest - the reason this was first
        implemented backwards.
        """
        theta, phi = math.pi / 4, math.pi / 3
        feature = np.array([5.0, 0.0, 0.0])       # a point on the ground ahead
        e0 = self._eye(theta, phi)
        e1 = self._eye(theta, phi - 0.25)         # phi -= dy
        self.assertLess(self._screen_y(e1, feature), self._screen_y(e0, feature),
                        "drag down should move the scene down the image")

    def test_inverted_vertical_sign_is_wrong(self):
        """phi += dy lowers the eye but sends the scene the wrong way."""
        theta, phi = math.pi / 4, math.pi / 3
        feature = np.array([5.0, 0.0, 0.0])
        e0 = self._eye(theta, phi)
        e1 = self._eye(theta, phi + 0.25)
        self.assertGreater(self._screen_y(e1, feature), self._screen_y(e0, feature),
                           "phi += dy moves the scene up; that is the inverted feel")

    def test_inverted_horizontal_sign_is_wrong(self):
        """The bug this guards: theta -= dx feels backwards in a Z-down frame."""
        theta, phi = math.pi / 4, math.pi / 3
        e0 = self._eye(theta, phi)
        e1 = self._eye(theta - 0.2, phi)
        self.assertGreater(float((e1 - e0) @ self._screen_right(e0)), 0,
                           "theta -= dx moves the eye right; that is the inverted feel")


@unittest.skipIf(np is None, "numpy not available")
class TestOverlayAgainstRealScene(unittest.TestCase):
    """Project real lidar returns and check they land on the correct cliff.

    overlay_scene_fixture.json holds returns captured from a live session in
    which the lidar was deliberately offset 2m to starboard. The points are
    classified by RANGE and HEIGHT only - the receding cliff is >40m away,
    the tall one is 8-18m away - so the test carries no assumption about
    which sign of y is left. In the camera image the receding cliff is on
    the left and the tall one on the right, and a correct projection must
    put each in its own half.

    This is the check that would have caught the mirror-in-Y: the synthetic
    tests above pass under a mirrored convention because their expected
    values were derived under the same convention.
    """

    @classmethod
    def setUpClass(cls):
        import json
        path = os.path.join(os.path.dirname(__file__), 'overlay_scene_fixture.json')
        with open(path) as f:
            cls.fix = json.load(f)

    def _project_all(self, points, lidar_loc, **kw):
        global LIDAR_LOC
        saved = LIDAR_LOC
        try:
            LIDAR_LOC = lidar_loc
            out = [project(np.array(p, float), **kw) for p in points]
        finally:
            LIDAR_LOC = saved
        return [p for p in out if p is not None]

    def test_far_left_cliff_lands_in_left_half(self):
        pts = self._project_all(self.fix['far_left_cliff'], self.fix['lidar_location'])
        self.assertTrue(pts)
        mean_x = sum(p[0] for p in pts) / len(pts)
        self.assertLess(mean_x, IMG_W / 2, f"far cliff drawn right (x={mean_x:.0f})")

    def test_close_right_cliff_lands_in_right_half(self):
        pts = self._project_all(self.fix['close_right_cliff'], self.fix['lidar_location'])
        self.assertTrue(pts)
        mean_x = sum(p[0] for p in pts) / len(pts)
        self.assertGreater(mean_x, IMG_W / 2, f"close cliff drawn left (x={mean_x:.0f})")

    def test_mirror_in_y_swaps_the_cliffs(self):
        """The shipped bug: with the mirror, each cliff lands on the wrong side."""
        far = self._project_all(self.fix['far_left_cliff'], self.fix['lidar_location'], mirror_y=True)
        close = self._project_all(self.fix['close_right_cliff'], self.fix['lidar_location'], mirror_y=True)
        far_x = sum(p[0] for p in far) / len(far)
        close_x = sum(p[0] for p in close) / len(close)
        self.assertGreater(far_x, IMG_W / 2, "mirror should push the far cliff right")
        self.assertLess(close_x, IMG_W / 2, "mirror should push the close cliff left")
