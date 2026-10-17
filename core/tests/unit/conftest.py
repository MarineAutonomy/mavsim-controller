"""Shared fixtures for the bridge core unit tests."""

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))


@pytest.fixture(autouse=True)
def _isolated_child_registry(tmp_path, monkeypatch):
    """BaseController() stops every helper recorded in CHILD_REGISTRY_FILE.
    Point it at an empty temporary file so constructing a controller in a test
    can never stop the helpers of a controller really running on this machine."""
    import base_controller
    monkeypatch.setattr(base_controller, 'CHILD_REGISTRY_FILE', str(tmp_path / 'child_processes.json'))
