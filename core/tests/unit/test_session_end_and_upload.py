#!/usr/bin/env python3
"""
Session end and bag upload (2026-11-23).

  - The bridge ran on after its session was stopped. It only noticed the end
    through a refused control command, and it read that flag off the
    MavsimController, which did not have it, so it never noticed at all; a
    bridge with no node commanding it sends no commands anyway. It now also
    asks the backend for the session's status every few seconds.
  - Bag uploads were cut off: they ran in a daemon thread that died with the
    process. close() now waits for them, up to a limit. Uploads are off
    unless asked for (start.sh --upload).

    python3 -m pytest core/tests/unit/test_session_end_and_upload.py
"""

import logging
import os
import sys
import threading
import time
from unittest.mock import Mock, patch

import pytest
import requests

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT)

import base_controller  # noqa: E402
import bridge_webapp  # noqa: E402
from base_controller import BaseController  # noqa: E402
from python_controller import MavsimAPIClient, MavsimController  # noqa: E402


def response(status, body):
    r = Mock()
    r.status_code = status
    if isinstance(body, Exception):
        r.json.side_effect = body
    else:
        r.json.return_value = body
    return r


def client():
    return MavsimAPIClient(backend_url='http://backend:5000', session_id='sess-1',
                           api_token='tok-1', vessel_name='sookshma_01')


def controller_over(api_client):
    """A MavsimController around `api_client`, without its connections."""
    ctrl = MavsimController.__new__(MavsimController)
    ctrl.api_client = api_client
    return ctrl


class TestSessionStatusCheck:
    @pytest.mark.parametrize('status', ['running', 'waiting_for_controller', 'queued'])
    def test_live_statuses_do_not_end_it(self, status):
        c = client()
        with patch('python_controller.requests.get', return_value=response(200, {'status': status})) as get:
            assert c.check_session_status() == status
        assert not c.session_ended
        assert get.call_args.kwargs['params'] == {'session_id': 'sess-1', 'api_token': 'tok-1'}
        assert get.call_args.args[0] == 'http://backend:5000/api/simulation/observer/status'

    @pytest.mark.parametrize('status', ['stopped', 'error'])
    def test_stopped_or_error_ends_it(self, status):
        c = client()
        with patch('python_controller.requests.get', return_value=response(200, {'status': status})):
            c.check_session_status()
        assert c.session_ended and status in c.session_end_reason

    def test_session_gone_ends_it(self):
        c = client()
        with patch('python_controller.requests.get',
                   return_value=response(404, {'error': 'Session not found'})):
            c.check_session_status()
        assert c.session_ended

    def test_other_404s_do_not(self):
        c = client()
        with patch('python_controller.requests.get', return_value=response(404, ValueError('html'))):
            assert c.check_session_status() is None
        assert not c.session_ended

    def test_unreachable_backend_does_not(self):
        c = client()
        with patch('python_controller.requests.get', side_effect=requests.ConnectionError('down')):
            assert c.check_session_status() is None
        assert not c.session_ended


class TestBaseControllerSeesTheEnd:
    def test_mavsim_controller_exposes_its_clients_flag(self):
        api = client()
        ctrl = controller_over(api)
        assert ctrl.session_ended is False
        api._mark_session_ended('session status is stopped')
        assert ctrl.session_ended is True
        assert ctrl.session_end_reason == 'session status is stopped'

    def test_base_controller_reads_it(self):
        """The bug: _session_ended() read MavsimController.session_ended,
        which did not exist, so it was always False."""
        api = client()
        c = BaseController(controller_code='TEST123')
        c._controllers = {'sookshma_01': controller_over(api)}
        assert c._session_ended() is False
        api._mark_session_ended('Session not running')
        assert c._session_ended() is True

    def test_poll_loop_checks_the_session(self, monkeypatch):
        c = BaseController(controller_code='TEST123')
        ctrl = Mock()
        c._controller = ctrl
        monkeypatch.setattr(BaseController, 'SESSION_CHECK_PERIOD', 0.0)
        monkeypatch.setattr(base_controller.time, 'sleep', lambda s: setattr(c, '_polling_active', False))
        with patch.object(c, 'poll_recording_command', return_value=None):
            c._polling_active = True
            c._poll_recording_loop()
        ctrl.check_session_status.assert_called_once()

    def test_control_loop_stops_when_the_check_ends_the_session(self):
        api = client()
        ctrl = controller_over(api)
        ctrl.get_vessel_states = lambda: {}          # nothing commanding, no states
        ctrl.clear_vessel_states = lambda: None
        c = BaseController(controller_code='TEST123')
        c._controller = ctrl
        c._controllers = {'sookshma_01': ctrl}
        c._control_rate_hz = 100.0
        c._running = True
        t = threading.Thread(target=c._control_loop, daemon=True)
        t.start()
        with patch('python_controller.requests.get', return_value=response(200, {'status': 'stopped'})):
            c._check_session_status()
        t.join(1.0)
        assert not t.is_alive() and c._running is False


class TestUpload:
    def recording_controller(self, monkeypatch, upload='1', timeout='120'):
        monkeypatch.setenv('MAVSIM_UPLOAD_BAGS', upload)
        monkeypatch.setenv('MAVSIM_UPLOAD_TIMEOUT', timeout)
        c = BaseController(controller_code='TEST123')
        c._api_client = Mock(session_id='sess-1', api_token='tok-1')
        c._recording = True
        c._recording_service = Mock()
        c._recording_service.stop_recording.return_value = '/tmp/mavsim_bags/mavsim_x'
        monkeypatch.setattr(c, '_report_recording_status', lambda *a, **k: None)
        return c

    def test_off_by_default(self, monkeypatch):
        monkeypatch.delenv('MAVSIM_UPLOAD_BAGS', raising=False)
        assert BaseController(controller_code='TEST123').upload_bags is False

    def test_off_switch(self, monkeypatch):
        c = self.recording_controller(monkeypatch, upload='0')
        with patch.object(c, '_upload_bag_async') as upload:
            c.stop_recording()
        upload.assert_not_called()
        assert c._upload_thread is None

    def test_close_waits_for_the_upload(self, monkeypatch):
        c = self.recording_controller(monkeypatch)
        done = []
        monkeypatch.setattr(c, '_upload_bag_async', lambda path: (time.sleep(0.3), done.append(path)))
        c.close()
        assert done == ['/tmp/mavsim_bags/mavsim_x']

    def test_close_gives_up_after_the_time_limit(self, monkeypatch, caplog):
        c = self.recording_controller(monkeypatch, timeout='0.1')
        release = threading.Event()
        monkeypatch.setattr(c, '_upload_bag_async', lambda path: release.wait(5))
        start = time.monotonic()
        with caplog.at_level(logging.WARNING, logger='base_controller'):
            c.close()
        release.set()
        assert time.monotonic() - start < 3.0
        assert any('did not finish' in r.message and 'mavsim_x' in r.message for r in caplog.records)

    def test_polling_stops_before_recording_does(self, monkeypatch):
        """Otherwise the next poll sees "start" again and restarts recording."""
        c = self.recording_controller(monkeypatch, upload='0')
        order = []
        monkeypatch.setattr(c, '_stop_polling', lambda: order.append('polling'))
        real_stop = c.stop_recording
        monkeypatch.setattr(c, 'stop_recording', lambda: (order.append('recording'), real_stop()))
        c.close()
        assert order[:2] == ['polling', 'recording']


class TestWebappStopTimeout:
    def test_leaves_time_for_the_upload(self, monkeypatch):
        monkeypatch.setenv('MAVSIM_UPLOAD_BAGS', '1')
        monkeypatch.setenv('MAVSIM_UPLOAD_TIMEOUT', '120')
        assert bridge_webapp.BridgeState._stop_timeout() == 128.0

    def test_plain_8s_without_uploads(self, monkeypatch):
        monkeypatch.setenv('MAVSIM_UPLOAD_BAGS', '0')
        assert bridge_webapp.BridgeState._stop_timeout() == 8.0
        monkeypatch.delenv('MAVSIM_UPLOAD_BAGS')
        assert bridge_webapp.BridgeState._stop_timeout() == 8.0
