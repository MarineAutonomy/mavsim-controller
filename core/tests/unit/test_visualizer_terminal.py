#!/usr/bin/env python3
"""
Unit tests for the visualizer's opt-in Terminal tab (--enable-terminal).

The tab is a bash shell in the bridge container with no login, so it must be
absent unless switched on. When on, keystrokes go up as POSTs and output
comes back as Server-Sent Events; the shell ends when its tab goes away.
"""

import json
import os
import time

import pytest

import visualizer_server as vs


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(vs, '_terminal_enabled', True)
    # Short keepalives so a stream that has nothing to say still yields.
    monkeypatch.setattr(vs, 'TERMINAL_KEEPALIVE_S', 0.1)
    yield vs.app.test_client()
    vs.close_all_terminals()


def _open(client, **size):
    resp = client.post('/api/terminal', json=size or {'cols': 80, 'rows': 24})
    assert resp.status_code == 201, resp.get_json()
    return resp.get_json()['id']


def _read_until(stream, needle, timeout=10.0):
    """Collect SSE output from a terminal stream until it contains needle."""
    text = ''
    deadline = time.monotonic() + timeout
    for chunk in stream.response:
        for line in chunk.decode().splitlines():
            if line.startswith('data: '):
                text += json.loads(line[len('data: '):]) if line != 'data: {}' else ''
            elif line == 'event: exit':
                text += '\n<exit>'
        if needle in text or time.monotonic() > deadline:
            break
    return text


def _wait_reaped(pid, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.05)
    return False


def test_off_by_default():
    assert vs._terminal_enabled is False
    client = vs.app.test_client()
    page = client.get('/').get_data(as_text=True)
    assert 'const TERMINAL_ENABLED = false;' in page
    assert client.post('/api/terminal', json={}).status_code == 404
    assert client.post('/api/terminal/x/input', data=b'ls\n').status_code == 404


def test_page_shows_tab_when_enabled(client):
    assert 'const TERMINAL_ENABLED = true;' in client.get('/').get_data(as_text=True)


def test_runs_commands_and_streams_output(client):
    tid = _open(client)
    stream = client.get(f'/api/terminal/{tid}/stream', buffered=False)
    assert stream.mimetype == 'text/event-stream'
    # The typed line echoes as "$((40+2))"; only the shell's output says 42.
    assert client.post(f'/api/terminal/{tid}/input', data=b'echo mavsim_$((40+2))\n').status_code == 204
    assert 'mavsim_42' in _read_until(stream, 'mavsim_42')
    stream.close()


def test_resize_reaches_the_shell(client):
    tid = _open(client, cols=100, rows=30)
    stream = client.get(f'/api/terminal/{tid}/stream', buffered=False)
    client.post(f'/api/terminal/{tid}/input', data=b'stty size\n')
    assert '30 100' in _read_until(stream, '30 100')
    assert client.post(f'/api/terminal/{tid}/resize', json={'cols': 120, 'rows': 40}).status_code == 204
    client.post(f'/api/terminal/{tid}/input', data=b'stty size\n')
    assert '40 120' in _read_until(stream, '40 120')
    stream.close()


def test_exit_ends_the_stream_and_the_session(client):
    tid = _open(client)
    stream = client.get(f'/api/terminal/{tid}/stream', buffered=False)
    client.post(f'/api/terminal/{tid}/input', data=b'exit\n')
    assert _read_until(stream, '<exit>').endswith('<exit>')
    stream.close()
    assert tid not in vs._terminals
    assert client.post(f'/api/terminal/{tid}/input', data=b'ls\n').status_code == 404


def test_dropped_stream_ends_the_shell(client):
    tid = _open(client)
    pid = vs._terminals[tid].pid
    stream = client.get(f'/api/terminal/{tid}/stream', buffered=False)
    next(iter(stream.response))
    stream.close()  # what the server sees when the tab goes away
    assert tid not in vs._terminals
    assert _wait_reaped(pid)


def test_close_without_a_stream_ends_the_shell(client):
    tid = _open(client)
    pid = vs._terminals[tid].pid
    assert client.post(f'/api/terminal/{tid}/close').status_code == 204
    assert tid not in vs._terminals
    assert _wait_reaped(pid)


def test_one_stream_per_terminal(client):
    tid = _open(client)
    first = client.get(f'/api/terminal/{tid}/stream', buffered=False)
    next(iter(first.response))
    assert client.get(f'/api/terminal/{tid}/stream').status_code == 409
    first.close()


def test_session_limit(client):
    for _ in range(vs.TERMINAL_MAX_SESSIONS):
        _open(client)
    resp = client.post('/api/terminal', json={})
    assert resp.status_code == 429
