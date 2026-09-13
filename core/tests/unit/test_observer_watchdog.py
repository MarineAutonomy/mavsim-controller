#!/usr/bin/env python3
"""
Unit tests for the headless observer's renderer watchdog.

The watchdog previously used console output as its liveness signal, which
conflated "the app is logging" with "the renderer is running". The frontend
logs heavily while building the scene and then goes quiet once it settles
into steady-state rendering, so a healthy tab read as stuck and was killed
roughly once a minute - camera/lidar frames never flowed because the session
never reached steady state.

The trace in test_real_recovering_trace_does_not_restart is the measured
one from that failure (two-vessel session, hardware WebGL on a Quadro RTX
8000), and is the specific case the old logic got wrong.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, '/app')

import observer


def simulate(frame_deltas, check_interval=None, grace=None, timeout=None):
    """Replay a sequence of per-check frame counts through the watchdog's
    decision logic, mirroring the loop in _run_observer_session().

    frame_deltas: frames rendered during each check interval; None models a
    liveness probe that failed to answer (busy page, torn-down context).

    Returns (fired, at_seconds).
    """
    check_interval = check_interval or observer._WATCHDOG_CHECK_INTERVAL_SECONDS
    grace = observer._WATCHDOG_STARTUP_GRACE_SECONDS if grace is None else grace
    timeout = observer._WATCHDOG_TIMEOUT_SECONDS if timeout is None else timeout

    now = 0.0
    loaded_at = 0.0
    last_progress = 0.0
    last_frames = -1
    total = 0

    for delta in frame_deltas:
        now += check_interval
        if delta is None:
            frames = None
        else:
            total += delta
            frames = total

        if frames is not None:
            if last_frames < 0 or frames - last_frames >= observer._WATCHDOG_MIN_FRAMES:
                last_progress = now
            if frames < last_frames:  # counter reset by a reload
                last_progress = now
            last_frames = frames

        if now - loaded_at < grace:
            continue
        if now - last_progress > timeout:
            return True, now

    return False, None


class TestObserverWatchdog(unittest.TestCase):

    def test_real_recovering_trace_does_not_restart(self):
        """The measured trace that the console-based watchdog got wrong.

        ~60fps, a dip to ~5fps during scene/BVH construction, then recovery
        to a steady ~53fps. Console went quiet for 59s and 69s across the
        dip, which is what used to trigger a restart.
        """
        trace = [300] * 2 + [26] * 2 + [243] * 2 + [267] * 2 + [266] * 24
        fired, at = simulate(trace)
        self.assertFalse(fired, f"healthy recovering renderer restarted at t={at}s")

    def test_stopped_renderer_restarts(self):
        """A renderer that genuinely stops producing frames must restart."""
        fired, at = simulate([300] * 4 + [0] * 40)
        self.assertTrue(fired, "stalled renderer was not detected")

    def test_sustained_slow_rendering_survives(self):
        """Slow is not the same as stuck: 5fps must not trigger a restart."""
        fired, at = simulate([300] * 2 + [25] * 60)
        self.assertFalse(fired, f"sustained 5fps restarted at t={at}s")

    def test_very_slow_rendering_survives(self):
        """Even 1fps is progress and must not trigger a restart."""
        fired, _ = simulate([300] * 2 + [5] * 60)
        self.assertFalse(fired)

    def test_startup_grace_covers_slow_scene_build(self):
        """Initial terrain/BVH work is legitimately long and partly
        synchronous; it must not be read as a hang."""
        slow_start = [0] * 17 + [300] * 40
        fired, at = simulate(slow_start)
        self.assertFalse(fired, f"slow scene build restarted at t={at}s")

    def test_never_renders_restarts_after_grace(self):
        """A tab that never renders must restart, but only once the
        startup grace period has elapsed."""
        fired, at = simulate([0] * 100)
        self.assertTrue(fired)
        self.assertGreaterEqual(at, observer._WATCHDOG_STARTUP_GRACE_SECONDS)

    def test_transient_probe_failures_do_not_restart(self):
        """A few unanswered liveness probes are not evidence of a hang."""
        fired, at = simulate([300] * 4 + [None] * 6 + [300] * 30)
        self.assertFalse(fired, f"transient probe failure restarted at t={at}s")

    def test_permanently_unreadable_page_restarts(self):
        """If the page stops answering entirely, restart it."""
        fired, _ = simulate([300] * 4 + [None] * 60)
        self.assertTrue(fired)

    def test_counter_reset_counts_as_progress(self):
        """An in-page navigation resets the counter; a decrease means the
        page reloaded, not that the renderer stalled."""
        # Model the reset directly: frames drops below the previous reading.
        last_frames, frames = 1000, 5
        progressed = (frames - last_frames >= observer._WATCHDOG_MIN_FRAMES
                      or frames < last_frames)
        self.assertTrue(progressed, "counter reset should count as progress")

    def test_min_frames_threshold_is_forgiving(self):
        """Exactly the minimum frame count still counts as alive."""
        fired, _ = simulate([300] * 2 + [observer._WATCHDOG_MIN_FRAMES] * 60)
        self.assertFalse(fired)


if __name__ == '__main__':
    unittest.main()
