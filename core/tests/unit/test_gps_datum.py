"""Local GPS fixes are about the config's gps_datum (2026-11-23).

The sensor generator took the datum from vesselConfig, the handshake's agents[] block,
which never carries it (gps_datum is a top-level config key): every fix was generated
about lat/lon (0, 0). The datum now comes from the handshake (gpsDatum) or the
fullConfig the bridge already fetches, and is passed to the generator explicitly.

    python3 -m pytest core/tests/unit/test_gps_datum.py
"""
import logging

import numpy as np
import pytest

import base_controller
from local_sensor_generator import LocalSensorGenerator

DATUM = [12.9915, 80.2337, 0.0]
AGENT = {   # an agents[] block as the handshake sends it: no gps_datum, no gravity
    'name': 'sookshma', 'vessel_id': '01',
    'sensors': [{'sensor_type': 'GPS', 'sensor_id': 2, 'sensor_topic': '/sookshma_01/gps_02/data',
                 'sensor_location': [0.8, 0.0, -0.178], 'publish_rate': 1.0,
                 'covariance': {'position_covariance': [1e-12, 0, 0, 0, 1e-12, 0, 0, 0, 1e-12]}}],
    'thrusters': [{'actuator_id': 1}, {'actuator_id': 2}],
}


def _state(x=5.0, y=5.0):
    s = np.zeros(14)          # u v w p q r x y z phi theta psi + 2 thrusters
    s[6], s[7] = x, y
    return s


def _fix(gen):
    return next(iter(gen.generate_once(_state()).values()))


def test_fix_is_about_the_session_datum():
    m = _fix(LocalSensorGenerator(AGENT, 'sookshma_01', gps_datum=DATUM))
    assert m['latitude'] == pytest.approx(DATUM[0], abs=1e-4)     # 5.8 m north: 5e-5 deg
    assert m['longitude'] == pytest.approx(DATUM[1], abs=1e-4)
    assert m['latitude'] > DATUM[0] and m['longitude'] > DATUM[1]


def test_without_a_datum_it_warns_and_uses_zero(caplog):
    with caplog.at_level(logging.WARNING):
        m = _fix(LocalSensorGenerator(AGENT, 'sookshma_01'))
    assert abs(m['latitude']) < 1e-3
    assert any('no GPS datum' in r.message for r in caplog.records)


def test_vessel_config_datum_still_works():
    m = _fix(LocalSensorGenerator({**AGENT, 'gps_datum': DATUM}, 'sookshma_01'))
    assert m['latitude'] == pytest.approx(DATUM[0], abs=1e-4)


def test_bridge_takes_datum_and_gravity_from_full_config(monkeypatch, tmp_path):
    class Resp:
        def raise_for_status(self):
            pass

        def json(self):
            return {'fullConfig': {'gps_datum': DATUM, 'gravity': 9.81, 'agents': [AGENT]}}

    import requests
    monkeypatch.setattr(requests, 'get', lambda *a, **k: Resp())
    monkeypatch.setattr(base_controller.os, 'makedirs', lambda *a, **k: None)

    class Fake:
        backend_url = 'http://backend'
        _wave_environment = {}
        _gps_datum = None
        _gravity = None

    real_open = open

    def fake_open(path, *a, **k):   # the visualizer state file goes to tmp_path
        return real_open(str(tmp_path / 'state.json') if 'mavsim_bridge_state' in str(path)
                         else path, *a, **k)
    monkeypatch.setattr('builtins.open', fake_open)
    fake = Fake()
    base_controller.BaseController._fetch_and_cache_sensor_config(fake, 'sess', 'tok', 'sookshma_01')
    assert fake._gps_datum == DATUM and fake._gravity == 9.81
