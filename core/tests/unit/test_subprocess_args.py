#!/usr/bin/env python3
"""
Subprocess arguments must survive values that begin with "-".

Session API tokens are URL-safe base64, so roughly one in thirty begins with
"-". Passed as two argv entries ("--api-token", "-Fhn..."), argparse reads the
value as the start of another option and exits:

    observer.py: error: argument --api-token: expected one argument

That killed the headless observer at startup, which is what streams camera and
lidar data, so those topics never appeared - intermittently, depending only on
how the token happened to be generated. The "--opt=value" form is unambiguous.
"""

import argparse
import os
import re
import sys
import unittest

CORE_DIR = os.path.join(os.path.dirname(__file__), '../..')
REPO_ROOT = os.path.join(CORE_DIR, '..')
sys.path.insert(0, CORE_DIR)
sys.path.insert(0, '/app')

# A real token observed from the backend; it starts with "-".
LEADING_HYPHEN_TOKEN = "-FhnpLJ0ZnoQ0tDfBZybM_KSVBzCVPC9XJt4jjc1rUo"
SESSION_ID = "b8e70b5d-b568-4e5d-9b72-8aec389ee3ac"


def _observer_parser():
    """The observer's argument contract, mirrored."""
    p = argparse.ArgumentParser()
    p.add_argument('--session-id', required=True)
    p.add_argument('--api-token', required=True)
    p.add_argument('--namespace', default='')
    p.add_argument('--frontend-url', default='')
    return p


class TestArgparseHyphenValues(unittest.TestCase):

    def test_separate_args_fail_on_hyphen_token(self):
        """Demonstrates the original failure, so the fix has a baseline."""
        argv = ['--session-id', SESSION_ID,
                '--api-token', LEADING_HYPHEN_TOKEN,
                '--namespace', '/sim_x/', '--frontend-url', 'http://x:5173']
        with self.assertRaises(SystemExit):
            _observer_parser().parse_args(argv)

    def test_equals_form_accepts_hyphen_token(self):
        argv = [f'--session-id={SESSION_ID}',
                f'--api-token={LEADING_HYPHEN_TOKEN}',
                '--namespace=/sim_x/', '--frontend-url=http://x:5173']
        args = _observer_parser().parse_args(argv)
        self.assertEqual(args.api_token, LEADING_HYPHEN_TOKEN)
        self.assertEqual(args.session_id, SESSION_ID)

    def test_equals_form_accepts_empty_namespace(self):
        """An empty value is the other way the two-argv form goes wrong."""
        argv = [f'--session-id={SESSION_ID}',
                f'--api-token={LEADING_HYPHEN_TOKEN}',
                '--namespace=', '--frontend-url=http://x:5173']
        args = _observer_parser().parse_args(argv)
        self.assertEqual(args.namespace, '')


class TestLaunchersUseEqualsForm(unittest.TestCase):
    """Guard the call sites, not just the parser contract."""

    def _read(self, relpath):
        with open(os.path.join(REPO_ROOT, relpath)) as f:
            return f.read()

    def test_observer_launch_uses_equals_form(self):
        src = self._read('core/base_controller.py')
        launch = src[src.index('def _launch_observer'):]
        launch = launch[:launch.index('def _reset_visualizer_state_file')]
        self.assertIn('f"--api-token={api_token}"', launch)
        self.assertIn('f"--session-id={session_id}"', launch)
        self.assertNotIn('"--api-token", api_token', launch,
                         "two-argv form reintroduced; a token starting with "
                         "'-' will kill the observer at startup")

    def test_bridge_webapp_uses_equals_form_for_user_values(self):
        src = self._read('bridge_webapp.py')
        build = src[src.index('def _build_command'):]
        build = build[:build.index('def _read_output')]
        for bad in ('"--token", token_path',
                    '"--code",',
                    '"--vessel-name", vessel_name'):
            self.assertNotIn(bad, build,
                             f"{bad} uses the two-argv form for a user value")
        self.assertIn('f"--token={token_path}"', build)


if __name__ == '__main__':
    unittest.main()
