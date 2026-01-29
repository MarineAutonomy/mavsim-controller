"""
Unit tests for Task 1.4: Camera Sensor Server

Tests verify that the CameraSensorServer:
- Receives and unpacks binary camera frames correctly
- Invokes registered callbacks with correct parameters
- Handles high-throughput scenarios (100+ frames/sec)
- Detects frame drops via timestamp gaps
- Manages thread pool for non-blocking callback execution
"""

import asyncio
import pytest
import time
import threading
import websockets
from mavsim_sensor_bridge.servers.camera import CameraSensorServer
from mavsim_sensor_bridge.utils.binary import pack_camera_frame, BinaryMessageError


@pytest.mark.asyncio
async def test_camera_server_receives_frame():
    """Test that server receives frames and invokes callbacks correctly."""
    received = []
    
    def callback(vessel_id, camera_id, timestamp, jpeg_data):
        received.append((vessel_id, camera_id, timestamp, jpeg_data))
    
    server = CameraSensorServer(port=18765)
    server.on_frame(vessel_id=1, camera_id=1, callback=callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb'
            frame = pack_camera_frame(1, 1, time.time(), jpeg_data)
            await ws.send(frame)
            await asyncio.sleep(0.2)  # Give callback time to execute
        
        assert len(received) == 1
        assert received[0][0] == 1  # vessel_id
        assert received[0][1] == 1  # camera_id
        assert received[0][3] == jpeg_data  # jpeg_data
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_high_throughput():
    """Test 100 frames/sec for 2 seconds (200 frames total)."""
    count = [0]  # Use list to allow modification from callback
    
    def callback(vessel_id, camera_id, timestamp, jpeg_data):
        count[0] += 1
    
    server = CameraSensorServer(port=18765, max_workers=8)
    server.on_frame(1, 1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff' * 50000  # 50KB JPEG
            for i in range(200):
                frame = pack_camera_frame(1, 1, time.time(), jpeg_data)
                await ws.send(frame)
                await asyncio.sleep(0.01)  # ~100 Hz
        
        # Wait for callbacks to complete
        await asyncio.sleep(0.5)
        
        # Allow some tolerance for timing (at least 190 out of 200)
        assert count[0] >= 190, f"Expected at least 190 frames, got {count[0]}"
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_multiple_vessels_cameras():
    """Test server handles multiple vessel/camera combinations."""
    received_v1_c1 = []
    received_v1_c2 = []
    received_v2_c1 = []
    
    def callback_v1_c1(vessel_id, camera_id, timestamp, jpeg_data):
        received_v1_c1.append((vessel_id, camera_id))
    
    def callback_v1_c2(vessel_id, camera_id, timestamp, jpeg_data):
        received_v1_c2.append((vessel_id, camera_id))
    
    def callback_v2_c1(vessel_id, camera_id, timestamp, jpeg_data):
        received_v2_c1.append((vessel_id, camera_id))
    
    server = CameraSensorServer(port=18765)
    server.on_frame(vessel_id=1, camera_id=1, callback=callback_v1_c1)
    server.on_frame(vessel_id=1, camera_id=2, callback=callback_v1_c2)
    server.on_frame(vessel_id=2, camera_id=1, callback=callback_v2_c1)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff'
            
            # Send frames for each combination
            await ws.send(pack_camera_frame(1, 1, time.time(), jpeg_data))
            await ws.send(pack_camera_frame(1, 2, time.time(), jpeg_data))
            await ws.send(pack_camera_frame(2, 1, time.time(), jpeg_data))
            
            await asyncio.sleep(0.2)
        
        assert len(received_v1_c1) == 1
        assert received_v1_c1[0] == (1, 1)
        assert len(received_v1_c2) == 1
        assert received_v1_c2[0] == (1, 2)
        assert len(received_v2_c1) == 1
        assert received_v2_c1[0] == (2, 1)
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_no_callback():
    """Test server handles frames when no callback is registered."""
    server = CameraSensorServer(port=18765)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff'
            frame = pack_camera_frame(1, 1, time.time(), jpeg_data)
            await ws.send(frame)
            await asyncio.sleep(0.1)
        
        # Should not crash - just log that no callback is registered
        assert True
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_invalid_message():
    """Test server handles invalid binary messages gracefully."""
    server = CameraSensorServer(port=18765)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            # Send invalid data (too short)
            await ws.send(b'invalid')
            await asyncio.sleep(0.1)
        
        # Should not crash - should log warning
        assert True
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_frame_drop_detection():
    """Test frame drop detection via timestamp gaps."""
    received_timestamps = []
    
    def callback(vessel_id, camera_id, timestamp, jpeg_data):
        received_timestamps.append(timestamp)
    
    server = CameraSensorServer(port=18765)
    server.on_frame(1, 1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff'
            base_time = time.time()
            
            # Send frame 1
            await ws.send(pack_camera_frame(1, 1, base_time, jpeg_data))
            await asyncio.sleep(0.05)
            
            # Send frame 2 with small gap (should not be detected as drop)
            await ws.send(pack_camera_frame(1, 1, base_time + 0.1, jpeg_data))
            await asyncio.sleep(0.05)
            
            # Send frame 3 with large gap (should be detected as drop)
            await ws.send(pack_camera_frame(1, 1, base_time + 1.0, jpeg_data))
            await asyncio.sleep(0.2)
        
        # Check that dropped frames were detected
        dropped = server.get_dropped_frames(1, 1)
        assert dropped >= 1, f"Expected at least 1 dropped frame, got {dropped}"
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_callback_error_handling():
    """Test that callback errors don't crash the server."""
    call_count = [0]
    
    def callback_with_error(vessel_id, camera_id, timestamp, jpeg_data):
        call_count[0] += 1
        if call_count[0] == 1:
            raise ValueError("Test error")
    
    server = CameraSensorServer(port=18765)
    server.on_frame(1, 1, callback_with_error)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff'
            
            # Send first frame (will raise error)
            await ws.send(pack_camera_frame(1, 1, time.time(), jpeg_data))
            await asyncio.sleep(0.1)
            
            # Send second frame (should still work)
            await ws.send(pack_camera_frame(1, 1, time.time(), jpeg_data))
            await asyncio.sleep(0.1)
        
        # Both frames should have been processed (callback called twice)
        assert call_count[0] == 2
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


def test_camera_server_on_frame_validation():
    """Test that on_frame validates input parameters."""
    server = CameraSensorServer(port=18765)
    
    # Invalid vessel_id
    with pytest.raises(ValueError, match="vessel_id"):
        server.on_frame(vessel_id=-1, camera_id=1, callback=lambda *args: None)
    
    with pytest.raises(ValueError, match="vessel_id"):
        server.on_frame(vessel_id=256, camera_id=1, callback=lambda *args: None)
    
    # Invalid camera_id
    with pytest.raises(ValueError, match="camera_id"):
        server.on_frame(vessel_id=1, camera_id=-1, callback=lambda *args: None)
    
    with pytest.raises(ValueError, match="camera_id"):
        server.on_frame(vessel_id=1, camera_id=256, callback=lambda *args: None)
    
    # Invalid callback
    with pytest.raises(TypeError, match="callable"):
        server.on_frame(vessel_id=1, camera_id=1, callback=None)


def test_camera_server_remove_callback():
    """Test removing callbacks."""
    server = CameraSensorServer(port=18765)
    
    def callback(vessel_id, camera_id, timestamp, jpeg_data):
        pass
    
    # Register callback
    server.on_frame(1, 1, callback)
    
    # Remove callback
    server.remove_callback(1, 1)
    
    # Should not raise error if removed again
    server.remove_callback(1, 1)


def test_camera_server_reset_dropped_frames():
    """Test resetting dropped frame counters."""
    server = CameraSensorServer(port=18765)
    
    # Initialize tracking for a vessel/camera
    server.on_frame(1, 1, lambda *args: None)
    
    # Manually set dropped frames (simulating detection)
    with server._sequence_lock:
        server.dropped_frames[(1, 1)] = 5
    
    assert server.get_dropped_frames(1, 1) == 5
    
    # Reset specific vessel/camera
    server.reset_dropped_frames(1, 1)
    assert server.get_dropped_frames(1, 1) == 0
    
    # Reset all
    with server._sequence_lock:
        server.dropped_frames[(1, 1)] = 3
        server.dropped_frames[(2, 1)] = 2
    
    server.reset_dropped_frames()
    assert server.get_dropped_frames(1, 1) == 0
    assert server.get_dropped_frames(2, 1) == 0


@pytest.mark.asyncio
async def test_camera_server_thread_pool_execution():
    """Test that callbacks execute in thread pool (non-blocking)."""
    callback_thread_ids = []
    callback_lock = threading.Lock()
    
    def callback(vessel_id, camera_id, timestamp, jpeg_data):
        thread_id = threading.current_thread().ident
        with callback_lock:
            callback_thread_ids.append(thread_id)
    
    server = CameraSensorServer(port=18765, max_workers=2)
    server.on_frame(1, 1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff'
            
            # Send multiple frames
            for _ in range(5):
                await ws.send(pack_camera_frame(1, 1, time.time(), jpeg_data))
                await asyncio.sleep(0.05)
            
            # Wait for callbacks to complete
            await asyncio.sleep(0.3)
        
        # Verify callbacks ran in worker threads (different from main thread)
        assert len(callback_thread_ids) == 5
        # All callbacks should run in worker threads (not the event loop thread)
        main_thread_id = threading.current_thread().ident
        for thread_id in callback_thread_ids:
            assert thread_id != main_thread_id, "Callback should run in worker thread"
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_camera_server_stats():
    """Test that server tracks statistics correctly."""
    server = CameraSensorServer(port=18765)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18765") as ws:
            jpeg_data = b'\xff\xd8\xff' * 100  # ~300 bytes
            
            # Send 5 frames
            for _ in range(5):
                await ws.send(pack_camera_frame(1, 1, time.time(), jpeg_data))
                await asyncio.sleep(0.05)
            
            # Check stats while connection is still open
            stats = server.get_stats()
            assert stats['messages'] == 5
            assert stats['bytes'] > 0  # Should have received bytes
            assert stats['connections'] == 1  # Connection still open
        
        # Wait for connection cleanup after context manager exits
        await asyncio.sleep(0.2)
        
        # Check stats after connection closed
        stats_after = server.get_stats()
        assert stats_after['messages'] == 5
        assert stats_after['bytes'] > 0
        assert stats_after['connections'] == 0  # Connection closed
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass
