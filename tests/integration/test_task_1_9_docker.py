"""
Integration tests for Task 1.9: Docker Development Environment

Tests verify that:
- Bridge starts in Docker container
- Bridge accepts external connections from test container
- WebSocket connections work across containers
- Health check mechanism works correctly
"""

import asyncio
import os
import pytest
import time
import websockets
from mavsim_sensor_bridge.utils.binary import pack_camera_frame


@pytest.mark.asyncio
async def test_bridge_accepts_external_connection():
    """Test that bridge in Docker accepts connections from host/test container."""
    bridge_host = os.environ.get('BRIDGE_HOST', 'localhost')
    bridge_port = 8765
    
    # Wait a moment for bridge to be fully ready
    await asyncio.sleep(0.5)
    
    try:
        async with websockets.connect(f"ws://{bridge_host}:{bridge_port}") as ws:
            # Send a test camera frame
            test_frame = pack_camera_frame(1, 1, time.time(), b'\xff\xd8\xff\xe0' + b'\x00' * 100)
            await ws.send(test_frame)
            # If no exception, connection works (successful send verifies connection is open)
    except Exception as e:
        pytest.fail(f"Failed to connect to bridge at {bridge_host}:{bridge_port}: {e}")


@pytest.mark.asyncio
async def test_bridge_multiple_connections():
    """Test that bridge accepts multiple concurrent connections."""
    bridge_host = os.environ.get('BRIDGE_HOST', 'localhost')
    bridge_port = 8765
    
    # Wait a moment for bridge to be fully ready
    await asyncio.sleep(0.5)
    
    # Create multiple connections
    connections = []
    try:
        for i in range(3):
            ws = await websockets.connect(f"ws://{bridge_host}:{bridge_port}")
            connections.append(ws)
        
        # Send frames from each connection (successful send verifies connection is open)
        for i, ws in enumerate(connections):
            test_frame = pack_camera_frame(i + 1, 1, time.time(), b'\xff\xd8\xff\xe0' + b'\x00' * 100)
            await ws.send(test_frame)
        
        # Close all connections
        for ws in connections:
            await ws.close()
    except Exception as e:
        # Clean up on error
        for ws in connections:
            try:
                await ws.close()
            except:
                pass
        pytest.fail(f"Failed to establish multiple connections: {e}")


@pytest.mark.asyncio
async def test_bridge_port_exposed():
    """Test that bridge WebSocket port is accessible."""
    bridge_host = os.environ.get('BRIDGE_HOST', 'localhost')
    bridge_port = 8765
    
    # Wait a moment for bridge to be fully ready
    await asyncio.sleep(0.5)
    
    try:
        async with websockets.connect(f"ws://{bridge_host}:{bridge_port}") as ws:
            # Send a frame to verify the connection is working
            test_frame = pack_camera_frame(1, 1, time.time(), b'\xff\xd8\xff\xe0' + b'\x00' * 50)
            await ws.send(test_frame)
            # If send succeeded, connection is working
    except ConnectionRefusedError:
        pytest.fail(f"Bridge port {bridge_port} is not accessible at {bridge_host}")
    except Exception as e:
        pytest.fail(f"Unexpected error connecting to bridge: {e}")


@pytest.mark.asyncio
async def test_bridge_health_check():
    """Test that bridge accepts a WebSocket health check (full handshake, no raw TCP to avoid server ERROR logs)."""
    bridge_host = os.environ.get('BRIDGE_HOST', 'localhost')
    bridge_port = 8765
    url = f"ws://{bridge_host}:{bridge_port}"

    await asyncio.sleep(0.5)

    try:
        async with websockets.connect(url, open_timeout=2) as ws:
            # Connection opened and handshake completed; server will not log "opening handshake failed"
            pass
    except Exception as e:
        pytest.fail(f"Health check failed: could not connect to {url}: {e}")
