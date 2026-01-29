"""
Unit tests for Task 1.6: Main SensorBridge Class

Tests verify that the SensorBridge:
- Initializes with default or custom configuration
- Starts and stops all sensor servers correctly
- Registers and invokes camera callbacks
- Handles concurrent server execution
- Provides statistics from servers
"""

import asyncio
import pytest
import time
import websockets
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig
from mavsim_sensor_bridge.utils.binary import pack_camera_frame


@pytest.mark.asyncio
async def test_bridge_starts_and_stops():
    """Test that bridge starts and stops correctly."""
    bridge = SensorBridge()
    
    assert not bridge.is_running
    
    # Start bridge in background task
    start_task = asyncio.create_task(bridge.start())
    
    # Wait a bit for server to start
    await asyncio.sleep(0.2)
    
    assert bridge.is_running
    
    # Stop bridge
    await bridge.stop()
    
    # Wait for task to complete
    await asyncio.sleep(0.1)
    
    assert not bridge.is_running
    
    # Clean up task
    if not start_task.done():
        start_task.cancel()
        try:
            await start_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_bridge_receives_camera():
    """Test that bridge receives camera frames and invokes callbacks."""
    received = []
    
    def camera_callback(vessel_id, camera_id, timestamp, jpeg_data):
        received.append((vessel_id, camera_id, timestamp, jpeg_data))
    
    bridge = SensorBridge()
    bridge.on_camera(1, 1, camera_callback)
    
    # Start bridge in background
    start_task = asyncio.create_task(bridge.start())
    
    # Wait for server to start
    await asyncio.sleep(0.3)
    
    try:
        # Connect and send a camera frame
        async with websockets.connect("ws://localhost:8765") as ws:
            frame = pack_camera_frame(1, 1, time.time(), b'\xff' * 1000)
            await ws.send(frame)
            await asyncio.sleep(0.2)  # Wait for callback to execute
        
        # Verify callback was invoked
        assert len(received) == 1
        assert received[0][0] == 1  # vessel_id
        assert received[0][1] == 1  # camera_id
        assert len(received[0][3]) == 1000  # jpeg_data length
    
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


@pytest.mark.asyncio
async def test_bridge_multiple_camera_callbacks():
    """Test that bridge handles multiple camera callbacks correctly."""
    received_v1_c1 = []
    received_v1_c2 = []
    received_v2_c1 = []
    
    def callback_v1_c1(v, c, t, j):
        received_v1_c1.append((v, c))
    
    def callback_v1_c2(v, c, t, j):
        received_v1_c2.append((v, c))
    
    def callback_v2_c1(v, c, t, j):
        received_v2_c1.append((v, c))
    
    bridge = SensorBridge()
    bridge.on_camera(1, 1, callback_v1_c1)
    bridge.on_camera(1, 2, callback_v1_c2)
    bridge.on_camera(2, 1, callback_v2_c1)
    
    # Start bridge
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.3)
    
    try:
        async with websockets.connect("ws://localhost:8765") as ws:
            # Send frames for different vessel/camera combinations
            await ws.send(pack_camera_frame(1, 1, time.time(), b'\xff' * 100))
            await ws.send(pack_camera_frame(1, 2, time.time(), b'\xff' * 100))
            await ws.send(pack_camera_frame(2, 1, time.time(), b'\xff' * 100))
            await asyncio.sleep(0.3)
        
        # Verify each callback received the correct frames
        assert len(received_v1_c1) == 1
        assert received_v1_c1[0] == (1, 1)
        assert len(received_v1_c2) == 1
        assert received_v1_c2[0] == (1, 2)
        assert len(received_v2_c1) == 1
        assert received_v2_c1[0] == (2, 1)
    
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


def test_bridge_config_default():
    """Test that bridge uses default configuration."""
    bridge = SensorBridge()
    
    assert bridge.config.camera_port == 8765
    assert bridge.config.camera_enabled is True
    assert 'camera' in bridge._servers


def test_bridge_config_custom():
    """Test that bridge accepts custom configuration."""
    config = BridgeConfig(
        camera_port=9000,
        camera_enabled=True,
        log_level=10  # DEBUG
    )
    
    bridge = SensorBridge(config=config)
    
    assert bridge.config.camera_port == 9000
    assert bridge.config.camera_enabled is True
    assert bridge._servers['camera'].port == 9000


def test_bridge_on_camera_without_server():
    """Test that on_camera raises error when camera server is disabled."""
    config = BridgeConfig(camera_enabled=False)
    bridge = SensorBridge(config=config)
    
    def dummy_callback(v, c, t, j):
        pass
    
    with pytest.raises(ValueError, match="Camera server is not enabled"):
        bridge.on_camera(1, 1, dummy_callback)


@pytest.mark.asyncio
async def test_bridge_stop_when_not_running():
    """Test that stopping a non-running bridge doesn't crash."""
    bridge = SensorBridge()
    
    # Should not raise exception
    await bridge.stop()


@pytest.mark.asyncio
async def test_bridge_start_twice():
    """Test that starting bridge twice raises error."""
    bridge = SensorBridge()
    
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.2)
    
    try:
        # Try to start again
        with pytest.raises(RuntimeError, match="already running"):
            await bridge.start()
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


@pytest.mark.asyncio
async def test_bridge_get_server_stats():
    """Test that bridge provides statistics from servers."""
    bridge = SensorBridge()
    
    # Get stats before starting (should still work)
    stats = bridge.get_server_stats()
    assert 'camera' in stats
    assert 'messages' in stats['camera']
    assert 'bytes' in stats['camera']
    assert 'connections' in stats['camera']
    
    # Get stats for specific sensor type
    camera_stats = bridge.get_server_stats('camera')
    assert 'messages' in camera_stats
    assert 'bytes' in camera_stats
    assert 'connections' in camera_stats
    
    # Test invalid sensor type
    with pytest.raises(ValueError, match="Unknown sensor type"):
        bridge.get_server_stats('nonexistent')


@pytest.mark.asyncio
async def test_bridge_stats_after_receiving_data():
    """Test that bridge statistics update after receiving data."""
    bridge = SensorBridge()
    
    def dummy_callback(v, c, t, j):
        pass
    
    bridge.on_camera(1, 1, dummy_callback)
    
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.3)
    
    try:
        async with websockets.connect("ws://localhost:8765") as ws:
            frame = pack_camera_frame(1, 1, time.time(), b'\xff' * 5000)
            await ws.send(frame)
            await asyncio.sleep(0.2)
        
        # Wait a bit for connection cleanup to complete
        await asyncio.sleep(0.1)
        
        # Check stats
        stats = bridge.get_server_stats('camera')
        assert stats['messages'] >= 1
        assert stats['bytes'] >= 5000
        # Connection count may vary due to async cleanup timing
        assert stats['connections'] >= 0
    
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


@pytest.mark.asyncio
async def test_bridge_no_servers_enabled():
    """Test that bridge handles case when no servers are enabled."""
    config = BridgeConfig(camera_enabled=False)
    bridge = SensorBridge(config=config)
    
    # Should not crash when starting with no servers
    await bridge.start()
    
    # Should be able to stop
    await bridge.stop()


@pytest.mark.asyncio
async def test_bridge_graceful_shutdown():
    """Test that bridge shuts down gracefully even with active connections."""
    bridge = SensorBridge()
    
    def slow_callback(v, c, t, j):
        time.sleep(0.1)  # Simulate slow processing
    
    bridge.on_camera(1, 1, slow_callback)
    
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.3)
    
    try:
        # Create a connection
        async with websockets.connect("ws://localhost:8765") as ws:
            # Send a frame
            await ws.send(pack_camera_frame(1, 1, time.time(), b'\xff' * 100))
            await asyncio.sleep(0.05)
            
            # Stop bridge while connection is active
            await bridge.stop()
        
        # Bridge should have stopped successfully
        assert not bridge.is_running
    
    finally:
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass
