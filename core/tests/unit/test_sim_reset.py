#!/usr/bin/env python3
"""
Unit tests for mid-session resets.

A reset sends every vessel back to its initial conditions at t = 0 while the
session and this connection carry on, so a controller can run trial after
trial (e.g. tuning PID gains). The simulator confirms each reset with
sim_reset_event (std_msgs/UInt32: resets so far). The control loop must then
drop the previous run's states and call on_reset() before control_loop()
sees the new run.
"""

import threading
import time
from unittest.mock import Mock, patch

from base_controller import BaseController
from python_controller import MavsimAPIClient, MavsimController


def _response(status_code, body):
    r = Mock()
    r.status_code = status_code
    r.json.return_value = body
    r.text = str(body)
    return r


def _client():
    return MavsimAPIClient(backend_url='http://backend:5000', session_id='sess-1',
                           api_token='tok-1', vessel_name='matsya_01')


@patch('python_controller.requests.post')
def test_api_client_posts_session_credentials(post):
    post.return_value = _response(200, {'success': True})
    assert _client().reset_simulation() is True
    url = post.call_args.args[0]
    assert url == 'http://backend:5000/api/control/reset'
    assert post.call_args.kwargs['json'] == {'sessionId': 'sess-1', 'apiToken': 'tok-1'}


@patch('python_controller.requests.post')
def test_api_client_reports_a_refused_reset(post):
    post.return_value = _response(404, {'error': 'Session not running'})
    assert _client().reset_simulation() is False


class Trials(BaseController):
    """Counts resets and records what control_loop() saw."""

    def __init__(self):
        super().__init__(controller_code='TEST123', backend_url='http://localhost:5000')
        self.resets = []
        self.seen = []
        self.reset_after = None

    def on_reset(self, reset_count):
        self.resets.append(reset_count)

    def control_loop(self, vessel_states):
        t = vessel_states['matsya_01']
        self.seen.append(t)
        if self.reset_after is not None and t >= self.reset_after:
            self.reset_after = None
            self.reset_simulation(timeout=1.0)
        return {'th_01': 1000.0}


def _wired(states):
    """A Trials controller over a fake MavsimController whose states come
    from the list `states` (latest last)."""
    c = Trials()
    ctrl = Mock(spec=MavsimController)
    ctrl.session_ended = False
    ctrl.get_vessel_states.side_effect = lambda: {'matsya_01': states[-1]} if states else {}
    ctrl.clear_vessel_states.side_effect = states.clear
    # The simulator answers a reset request with sim_reset_event.
    ctrl.reset_simulation.side_effect = lambda: (
        threading.Timer(0.05, c._on_reset_event, args=(c._reset_count + 1,)).start() or True)
    c._controller = ctrl
    c._controllers = {'matsya_01': ctrl}
    c._control_rate_hz = 200.0
    return c, ctrl


def _run(c, seconds):
    c._running = True
    thread = threading.Thread(target=c._control_loop, daemon=True)
    thread.start()
    time.sleep(seconds)
    c._running = False
    thread.join(timeout=1.0)


def test_dashboard_reset_clears_states_then_calls_on_reset():
    states = [12.3]
    c, _ = _wired(states)
    c._on_reset_event(1)   # e.g. the dashboard's Reset button
    _run(c, 0.1)
    assert c.resets == [1]
    assert states == []    # the previous run's state was dropped
    assert c.seen == []    # and control_loop() never saw it


def test_reset_from_control_loop_drops_that_ticks_commands():
    states = [60.0]
    c, ctrl = _wired(states)
    c.reset_after = 60.0
    _run(c, 0.3)
    assert c.resets == [1]
    assert c.seen == [60.0]
    ctrl.send_command.assert_not_called()


def test_reset_without_confirmation_returns_false():
    c, ctrl = _wired([1.0])
    ctrl.reset_simulation.side_effect = lambda: True   # accepted, never confirmed
    assert c.reset_simulation(timeout=0.1) is False
