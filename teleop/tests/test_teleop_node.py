#!/usr/bin/env python3
"""
Teleop node robustness (2026-10-17).

A teleop page sometimes stopped responding: the previous session's teleop
node had outlived its controller, held ports 8900/8901 (so the new one failed
with "Address already in use") and its command loop had stopped, while its
page still took keys and sent telemetry. These pin the fixes:
  - the command loop survives errors instead of ending silently;
  - vessels written to the config after start-up are picked up;
  - the node exits when the controller that launched it is gone;
  - it refuses to start when its ports are taken.

No ROS needed (rclpy missing -> commands are computed, not published):
    python3 -m pytest teleop/tests/test_teleop_node.py
"""

import json
import os
import socket
import subprocess
import sys
import textwrap
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import teleop_node as tn  # noqa: E402

THRUSTER = {'actuator_id': 1, 'thruster_location': [-1.0, 0.0, 0.0], 'thruster_orientation': [0, 0, 0],
            'D_prop': 0.1, 'J_vs_KT': [[-1, 0, 1], [0.3, 0.15, 0.0]], 'n_max': 2668, 'n_min': -2668}


def write_config(path, vessels):
    with open(path, 'w') as f:
        json.dump({name: {'thrusters': [THRUSTER], 'control_surfaces': []} for name in vessels}, f)


def run_loop(server, seconds):
    t = threading.Thread(target=server.run_command_loop, daemon=True)
    t.start()
    time.sleep(seconds)
    server._stop = True
    t.join(2.0)
    return t


class TestCommandLoop(unittest.TestCase):
    def setUp(self):
        self.dir = os.path.join(os.path.dirname(__file__), '_tmp_teleop')
        os.makedirs(self.dir, exist_ok=True)
        self.config = os.path.join(self.dir, 'teleop_config.json')
        write_config(self.config, ['Mako_01'])

    def tearDown(self):
        for f in os.listdir(self.dir):
            os.remove(os.path.join(self.dir, f))
        os.rmdir(self.dir)

    def test_keeps_running_after_an_error(self):
        server = tn.TeleopServer(self.config, rate=50.0, ramp_seconds=0.5)
        vessel = server.vessels['Mako_01']
        calls = []

        def flaky(_actuator_cls):
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError("publisher's context is invalid")

        vessel.publish = flaky
        with self.assertLogs('teleop_node', level='ERROR') as logs:
            thread = run_loop(server, 0.4)
        self.assertFalse(thread.is_alive())
        self.assertGreater(len(calls), 5)          # carried on after the first failure
        self.assertTrue(any('continuing' in line for line in logs.output))

    def test_picks_up_vessels_added_after_start(self):
        server = tn.TeleopServer(self.config, rate=50.0, ramp_seconds=0.5)
        self.assertEqual(list(server.vessels), ['Mako_01'])
        time.sleep(0.05)
        write_config(self.config, ['Mako_01', 'dock_02'])
        os.utime(self.config, None)
        run_loop(server, 0.3)
        self.assertEqual(sorted(server.vessels), ['Mako_01', 'dock_02'])

    def test_existing_vessels_are_not_rebuilt(self):
        server = tn.TeleopServer(self.config, rate=50.0, ramp_seconds=0.5)
        original = server.vessels['Mako_01']
        write_config(self.config, ['Mako_01', 'dock_02'])
        run_loop(server, 0.3)
        self.assertIs(server.vessels['Mako_01'], original)    # held keys / ramp state kept


class TestPorts(unittest.TestCase):
    def test_busy_port_is_reported(self):
        holder = socket.socket()
        holder.bind(('0.0.0.0', 0))
        holder.listen(1)
        busy = holder.getsockname()[1]
        free_probe = socket.socket(); free_probe.bind(('0.0.0.0', 0)); free = free_probe.getsockname()[1]; free_probe.close()
        try:
            self.assertEqual(tn._busy_ports(busy, free), [busy])
        finally:
            holder.close()

    def test_refuses_to_start_on_busy_ports(self):
        holder = socket.socket()
        holder.bind(('0.0.0.0', 0))
        holder.listen(1)
        port = holder.getsockname()[1]
        try:
            proc = subprocess.run([sys.executable, tn.__file__, '--http-port', str(port), '--ws-port', str(port + 1),
                                   '--config-file', '/nonexistent'], capture_output=True, text=True, timeout=30)
        finally:
            holder.close()
        self.assertEqual(proc.returncode, 3)
        self.assertIn('already in use', proc.stderr + proc.stdout)


class TestExitWithParent(unittest.TestCase):
    def test_exits_when_its_parent_dies(self):
        teleop_dir = os.path.dirname(tn.__file__)
        child = textwrap.dedent(f"""
            import sys, time
            sys.path.insert(0, {teleop_dir!r})
            import teleop_node
            teleop_node._exit_with_parent(poll_seconds=0.1)
            time.sleep(30)
        """)
        parent = textwrap.dedent(f"""
            import subprocess, sys, time
            p = subprocess.Popen([sys.executable, '-c', {child!r}], start_new_session=True)
            print(p.pid, flush=True)
            time.sleep(30)
        """)
        outer = subprocess.Popen([sys.executable, '-c', parent], stdout=subprocess.PIPE, text=True)
        child_pid = int(outer.stdout.readline())
        time.sleep(1.0)                                # child is up and watching
        outer.kill(); outer.wait()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                os.kill(child_pid, 0)
            except ProcessLookupError:
                break
            if open(f'/proc/{child_pid}/stat').read().rsplit(')', 1)[1].split()[0] == 'Z':
                break
            time.sleep(0.1)
        else:
            os.kill(child_pid, 9)
            self.fail('teleop child kept running after its parent died')


if __name__ == '__main__':
    unittest.main()
