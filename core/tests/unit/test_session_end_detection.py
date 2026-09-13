#!/usr/bin/env python3
"""
Unit tests for detecting a finished simulation.

When a simulation ends (maximum simulation time reached, or stopped from the
UI) the backend starts refusing control commands:

    HTTP 404
    {"error": "Session not running",
     "message": "Session <uuid> is not in running status (current status: stopped)"}

That response shape is the one observed from a live backend. Previously it
was logged as a generic warning and otherwise ignored, so the control loop
kept posting commands at its full rate against a dead session indefinitely.
"""

import os
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, '/app')

from python_controller import MavsimAPIClient


def _response(status_code, body):
    r = Mock()
    r.status_code = status_code
    r.json.return_value = body
    r.text = str(body)
    return r


SESSION_ENDED_BODY = {
    "error": "Session not running",
    "message": ("Session 2699e8f6-bf21-429f-bbad-7521ca65628f is not in "
                "running status (current status: stopped)"),
}


class TestSessionEndDetection(unittest.TestCase):

    def _client(self):
        return MavsimAPIClient(
            backend_url='http://backend:5000',
            session_id='sess-1',
            api_token='tok-1',
            vessel_name='matsya_01',
        )

    def test_starts_not_ended(self):
        c = self._client()
        self.assertFalse(c.session_ended)
        self.assertIsNone(c.session_end_reason)

    @patch('python_controller.requests.post')
    def test_detects_session_ended_from_real_response(self, post):
        """The exact 404 body a live backend returns must be detected."""
        post.return_value = _response(404, SESSION_ENDED_BODY)
        c = self._client()
        ok = c.send_command(['cs_01'], [0.0])
        self.assertFalse(ok)
        self.assertTrue(c.session_ended)
        self.assertIn('not in running status', c.session_end_reason)

    @patch('python_controller.requests.post')
    def test_success_does_not_flag_ended(self, post):
        post.return_value = _response(200, {})
        c = self._client()
        self.assertTrue(c.send_command(['cs_01'], [0.0]))
        self.assertFalse(c.session_ended)

    @patch('python_controller.requests.post')
    def test_other_errors_do_not_flag_ended(self, post):
        """A 400/500 is a transient or caller error, not a finished sim."""
        for status, body in [
            (400, {"error": "Missing actuatorNames"}),
            (401, {"error": "Invalid token"}),
            (500, {"error": "Internal error"}),
        ]:
            post.return_value = _response(status, body)
            c = self._client()
            self.assertFalse(c.send_command(['cs_01'], [0.0]))
            self.assertFalse(c.session_ended, f"{status} must not end session")

    @patch('python_controller.requests.post')
    def test_404_unrelated_does_not_flag_ended(self, post):
        """A 404 that isn't about the session running must not end it."""
        post.return_value = _response(404, {"error": "Vessel not found"})
        c = self._client()
        self.assertFalse(c.send_command(['cs_01'], [0.0]))
        self.assertFalse(c.session_ended)

    @patch('python_controller.requests.post')
    def test_non_json_error_body_is_survivable(self, post):
        """An HTML/empty error body must not raise out of send_command."""
        r = Mock()
        r.status_code = 502
        r.json.side_effect = ValueError('no json')
        r.text = '<html>bad gateway</html>'
        post.return_value = r
        c = self._client()
        self.assertFalse(c.send_command(['cs_01'], [0.0]))
        self.assertFalse(c.session_ended)

    @patch('python_controller.requests.post')
    def test_flag_latches_and_reason_is_kept(self, post):
        """Once ended, a later transport error must not clear the flag."""
        post.return_value = _response(404, SESSION_ENDED_BODY)
        c = self._client()
        c.send_command(['cs_01'], [0.0])
        first_reason = c.session_end_reason

        import requests as _requests
        post.side_effect = _requests.exceptions.RequestException('connection reset')
        c.send_command(['cs_01'], [0.0])
        self.assertTrue(c.session_ended)
        self.assertEqual(c.session_end_reason, first_reason)

    @patch('python_controller.requests.post')
    def test_detection_is_case_insensitive(self, post):
        post.return_value = _response(404, {"error": "SESSION NOT RUNNING"})
        c = self._client()
        c.send_command(['cs_01'], [0.0])
        self.assertTrue(c.session_ended)

    @patch('python_controller.requests.post')
    def test_logs_once_not_every_call(self, post):
        """The end is logged once, not at the control rate."""
        post.return_value = _response(404, SESSION_ENDED_BODY)
        c = self._client()
        with self.assertLogs('python_controller', level='INFO') as cm:
            for _ in range(5):
                c.send_command(['cs_01'], [0.0])
        ended = [m for m in cm.output if 'session has ended' in m.lower()]
        self.assertEqual(len(ended), 1, f"expected one log, got {ended}")


class TestControllerStopsOnSessionEnd(unittest.TestCase):
    """The BaseController side: a finished session must stop the loop."""

    def _controller(self):
        from base_controller import BaseController
        ctrl = BaseController.__new__(BaseController)
        ctrl._controllers = {}
        ctrl._controller = None
        ctrl._running = True
        return ctrl

    def test_session_ended_false_when_no_clients(self):
        self.assertFalse(self._controller()._session_ended())

    def test_session_ended_sees_primary_client(self):
        ctrl = self._controller()
        client = Mock()
        client.session_ended = True
        client.session_end_reason = 'max time reached'
        ctrl._controller = client
        self.assertTrue(ctrl._session_ended())

    def test_session_ended_sees_any_vessel_client(self):
        """Multi-vessel: only the vessels commanded this tick see the 404."""
        ctrl = self._controller()
        a, b = Mock(), Mock()
        a.session_ended, a.session_end_reason = False, None
        b.session_ended, b.session_end_reason = True, 'stopped'
        ctrl._controllers = {'v1': a, 'v2': b}
        self.assertTrue(ctrl._session_ended())

    def test_running_clients_do_not_trigger_stop(self):
        ctrl = self._controller()
        a = Mock()
        a.session_ended, a.session_end_reason = False, None
        ctrl._controllers = {'v1': a}
        self.assertFalse(ctrl._session_ended())

    def test_handle_session_ended_clears_running(self):
        ctrl = self._controller()
        client = Mock()
        client.session_ended = True
        client.session_end_reason = 'maximum simulation time reached'
        ctrl._controller = client
        ctrl._handle_session_ended()
        self.assertFalse(ctrl._running,
                         "_running must go False so run() returns and close() runs")

    def test_handle_session_ended_without_reason(self):
        """A missing reason must not break the shutdown path."""
        ctrl = self._controller()
        client = Mock()
        client.session_ended = True
        client.session_end_reason = None
        ctrl._controller = client
        ctrl._handle_session_ended()
        self.assertFalse(ctrl._running)

    def test_all_clients_deduplicates_primary(self):
        """The primary is often also in _controllers; don't double-count."""
        ctrl = self._controller()
        client = Mock()
        client.session_ended = False
        ctrl._controllers = {'v1': client}
        ctrl._controller = client
        self.assertEqual(len(ctrl._all_clients()), 1)


if __name__ == '__main__':
    unittest.main()
