"""GPS positions generated locally must match the WGS84 ellipsoid.

Until 2026-10-31 _ned_to_llh moved latitude with the prime vertical radius
R_N and longitude with the meridian radius R_M (swapped), and R_M lacked its
3/2 power: GPS readings were off by about 0.65 % of the distance from the
datum. The simulator's module_kinematics.ned_to_llh had the same error and was
fixed in the same way.
"""
import numpy as np
import pytest

from local_sensor_generator import _ned_to_llh

A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)


def _radii(lat_deg):
    s = np.sin(np.radians(lat_deg))
    w = 1 - E2 * s * s
    return A * (1 - E2) / w ** 1.5, A / np.sqrt(w)   # R_M, R_N


@pytest.mark.parametrize('lat0', [0.0, 13.0, 45.0])
def test_north_and_east_follow_the_ellipsoid(lat0):
    rm, rn = _radii(lat0)
    llh = _ned_to_llh(np.array([2000.0, 3000.0, -4.0]), np.array([lat0, 80.0, 1.0]))
    assert np.radians(llh[0] - lat0) * rm == pytest.approx(2000.0, abs=0.01)
    assert np.radians(llh[1] - 80.0) * rn * np.cos(np.radians(lat0)) == pytest.approx(3000.0, abs=0.01)
    assert llh[2] == pytest.approx(5.0)


def test_one_degree_of_latitude_at_the_equator():
    # WGS84: 110 574 m per degree of latitude at the equator, centred on it.
    llh = _ned_to_llh(np.array([110574.3, 0.0, 0.0]), np.array([-0.5, 0.0, 0.0]))
    assert llh[0] == pytest.approx(0.5, abs=2e-4)
