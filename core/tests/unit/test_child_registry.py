#!/usr/bin/env python3
"""
Helper-process registry (2026-10-17).

base_controller.py starts its helpers (observer, rosbridge, visualizer,
teleop) in their own sessions, so stopping the bridge never reached them when
the controller's close() was cut short; an orphaned teleop then held its
ports and broke the next session's teleop. Helpers are now recorded by pid +
kernel start time, and kill_registered_children() - in base_controller and
its twin in bridge_webapp - stops whatever survived. Both twins are tested.

    python3 -m pytest core/tests/unit/test_child_registry.py
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'core'))
sys.path.insert(0, ROOT)

import base_controller  # noqa: E402
import bridge_webapp  # noqa: E402

SLEEPER = [sys.executable, '-c', 'import time; time.sleep(60)']


def alive(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


class RegistryCases:
    module = None

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.registry = os.path.join(self.tmp, 'child_processes.json')
        self._saved = (base_controller.CHILD_REGISTRY_FILE, bridge_webapp.CHILD_REGISTRY_FILE)
        base_controller.CHILD_REGISTRY_FILE = self.registry
        bridge_webapp.CHILD_REGISTRY_FILE = self.registry
        self.procs = []

    def tearDown(self):
        base_controller.CHILD_REGISTRY_FILE, bridge_webapp.CHILD_REGISTRY_FILE = self._saved
        for p in self.procs:
            if p.poll() is None:
                p.kill()
            p.wait()
        for f in os.listdir(self.tmp):
            os.remove(os.path.join(self.tmp, f))
        os.rmdir(self.tmp)

    def spawn(self, cmd=SLEEPER, **kw):
        p = subprocess.Popen(cmd, start_new_session=True, **kw)
        self.procs.append(p)
        time.sleep(0.2)
        return p

    def test_kills_a_survivor_and_forgets_the_registry(self):
        p = self.spawn()
        base_controller.register_child_process('teleop', p)
        self.assertEqual(self.module.kill_registered_children(timeout=3.0), ['teleop'])
        p.wait(timeout=3)
        self.assertFalse(alive(p.pid))
        self.assertFalse(os.path.exists(self.registry))

    def test_kills_the_whole_process_group(self):
        # like `ros2 launch`, whose real workers are its own children
        code = 'import subprocess, sys, time; c = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]); print(c.pid, flush=True); time.sleep(60)'
        p = self.spawn([sys.executable, '-c', code], stdout=subprocess.PIPE, text=True)
        grandchild = int(p.stdout.readline())
        base_controller.register_child_process('rosbridge', p)
        self.module.kill_registered_children(timeout=3.0)
        p.wait(timeout=3)
        deadline = time.monotonic() + 3
        while alive(grandchild) and time.monotonic() < deadline:
            time.sleep(0.1)
        self.assertFalse(alive(grandchild))

    def test_spares_a_reused_pid(self):
        p = self.spawn()
        with open(self.registry, 'w') as f:      # same pid, different start time = a different process
            json.dump({'teleop': {'pid': p.pid, 'start': '1'}}, f)
        self.assertEqual(self.module.kill_registered_children(timeout=0.5), [])
        self.assertTrue(alive(p.pid))

    def test_zombies_and_missing_files_are_fine(self):
        p = self.spawn([sys.executable, '-c', 'pass'])
        base_controller.register_child_process('observer', p)   # may already be a zombie
        time.sleep(0.3)
        self.assertEqual(self.module.kill_registered_children(timeout=0.5), [])
        os.path.exists(self.registry) and os.remove(self.registry)
        self.assertEqual(self.module.kill_registered_children(timeout=0.5), [])


class TestBaseControllerTwin(RegistryCases, unittest.TestCase):
    module = base_controller


class TestBridgeWebappTwin(RegistryCases, unittest.TestCase):
    module = bridge_webapp


class TestLaunchSitesRegister(unittest.TestCase):
    def test_every_helper_launch_is_recorded(self):
        src = open(os.path.join(ROOT, 'core', 'base_controller.py')).read()
        for attr, label in [('_observer_process', 'observer'), ('_rosbridge_process', 'rosbridge'),
                            ('_visualizer_process', 'visualizer'), ('_teleop_process', 'teleop')]:
            self.assertIn(f'register_child_process("{label}", self.{attr})', src)
        self.assertEqual(src.count('subprocess.Popen(['), 4, 'a new helper launch needs register_child_process()')


if __name__ == '__main__':
    unittest.main()
