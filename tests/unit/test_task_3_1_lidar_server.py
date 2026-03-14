"""
Unit tests for Task 3.1: Lidar Sensor Server

Tests verify that the LidarSensorServer:
- Receives and unpacks binary lidar scans correctly
- Invokes registered callbacks with correct parameters
- Handles high-throughput scenarios
- Validates point count and format
- Manages thread pool for non-blocking callback execution
"""

import asyncio
import pytest
import time
import threading
import struct
import websockets
import numpy as np
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer
from mavsim_sensor_bridge.utils.binary import pack_lidar_scan, BinaryMessageError


@pytest.mark.asyncio
async def test_lidar_server_receives_scan():
    """Test that server receives scans and invokes callbacks correctly."""
    received = []
    
    def callback(vessel_id, lidar_id, points, timestamp):
        received.append((points, timestamp))
    
    server = LidarSensorServer(port=18766)
    server.on_scan(vessel_id=1, callback=callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points = np.random.randn(1000, 4).astype(np.float32)
            scan = pack_lidar_scan(1, time.time(), points)
            await ws.send(scan)
            await asyncio.sleep(0.2)  # Give callback time to execute
        
        assert len(received) == 1
        assert received[0][0].shape == (1000, 4)  # points
        assert received[0][0].dtype == np.float32
        np.testing.assert_array_equal(received[0][0], points)
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_lidar_server_high_throughput():
    """Test 10 scans/sec for 2 seconds (20 scans total)."""
    count = [0]  # Use list to allow modification from callback
    
    def callback(vessel_id, lidar_id, points, timestamp):
        count[0] += 1
    
    server = LidarSensorServer(port=18766, max_workers=8)
    server.on_scan(1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points = np.random.randn(10000, 4).astype(np.float32)  # 10K points per scan
            for i in range(20):
                scan = pack_lidar_scan(1, time.time(), points)
                await ws.send(scan)
                await asyncio.sleep(0.1)  # ~10 Hz
        
        # Wait for callbacks to complete
        await asyncio.sleep(0.5)
        
        # Allow some tolerance for timing (at least 18 out of 20)
        assert count[0] >= 18, f"Expected at least 18 scans, got {count[0]}"
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_lidar_server_multiple_vessels():
    """Test server handles multiple vessel combinations."""
    received_v1 = []
    received_v2 = []
    
    def callback_v1(vessel_id, lidar_id, points, timestamp):
        received_v1.append(points.shape[0])
    
    def callback_v2(vessel_id, lidar_id, points, timestamp):
        received_v2.append(points.shape[0])
    
    server = LidarSensorServer(port=18766)
    server.on_scan(vessel_id=1, callback=callback_v1)
    server.on_scan(vessel_id=2, callback=callback_v2)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points_v1 = np.random.randn(500, 4).astype(np.float32)
            points_v2 = np.random.randn(1000, 4).astype(np.float32)
            
            # Send scans for each vessel
            await ws.send(pack_lidar_scan(1, time.time(), points_v1))
            await ws.send(pack_lidar_scan(2, time.time(), points_v2))
            
            await asyncio.sleep(0.2)
        
        assert len(received_v1) == 1
        assert received_v1[0] == 500
        assert len(received_v2) == 1
        assert received_v2[0] == 1000
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_lidar_server_no_callback():
    """Test server handles scans when no callback is registered."""
    server = LidarSensorServer(port=18766)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points = np.random.randn(100, 4).astype(np.float32)
            scan = pack_lidar_scan(1, time.time(), points)
            await ws.send(scan)
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
async def test_lidar_server_invalid_message():
    """Test server handles invalid binary messages gracefully."""
    server = LidarSensorServer(port=18766)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
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
async def test_lidar_server_point_count_validation():
    """Test point count validation."""
    received = []
    
    def callback(vessel_id, lidar_id, points, timestamp):
        received.append(points.shape[0])
    
    server = LidarSensorServer(port=18766, max_points_per_scan=1000)
    server.on_scan(1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            # Send scan with too many points (should be truncated)
            points = np.random.randn(2000, 4).astype(np.float32)
            scan = pack_lidar_scan(1, time.time(), points)
            await ws.send(scan)
            await asyncio.sleep(0.2)
        
        # Should receive truncated scan
        assert len(received) == 1
        assert received[0] == 1000  # Truncated to max_points_per_scan
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_lidar_server_empty_scan():
    """Test server handles empty scans gracefully."""
    received = []
    
    def callback(vessel_id, lidar_id, points, timestamp):
        received.append(points)
    
    server = LidarSensorServer(port=18766)
    server.on_scan(1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            # Manually construct empty scan message (pack_lidar_scan doesn't allow empty arrays)
            # Format: vessel_id (uint8) + timestamp (float64) + point_count (uint32) + points
            vessel_id = 1
            timestamp = time.time()
            point_count = 0
            # Pack header: vessel_id (uint8), timestamp (float64), point_count (uint32)
            scan = struct.pack('>BdI', vessel_id, timestamp, point_count)
            await ws.send(scan)
            await asyncio.sleep(0.2)
        
        # Should not invoke callback for empty scan
        assert len(received) == 0
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_lidar_server_callback_error_handling():
    """Test that callback errors don't crash the server."""
    call_count = [0]
    
    def callback_with_error(vessel_id, lidar_id, points, timestamp):
        call_count[0] += 1
        if call_count[0] == 1:
            raise ValueError("Test error")
    
    server = LidarSensorServer(port=18766)
    server.on_scan(1, callback_with_error)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points = np.random.randn(100, 4).astype(np.float32)
            
            # Send first scan (will raise error)
            await ws.send(pack_lidar_scan(1, time.time(), points))
            await asyncio.sleep(0.1)
            
            # Send second scan (should still work)
            await ws.send(pack_lidar_scan(1, time.time(), points))
            await asyncio.sleep(0.1)
        
        # Both scans should have been processed (callback called twice)
        assert call_count[0] == 2
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


def test_lidar_server_on_scan_validation():
    """Test that on_scan validates input parameters."""
    server = LidarSensorServer(port=18766)
    
    # Invalid vessel_id
    with pytest.raises(ValueError, match="vessel_id"):
        server.on_scan(vessel_id=-1, callback=lambda *args: None)
    
    with pytest.raises(ValueError, match="vessel_id"):
        server.on_scan(vessel_id=256, callback=lambda *args: None)
    
    # Invalid callback
    with pytest.raises(TypeError, match="callable"):
        server.on_scan(vessel_id=1, callback=None)


def test_lidar_server_remove_callback():
    """Test removing callbacks."""
    server = LidarSensorServer(port=18766)
    
    def callback(vessel_id, lidar_id, points, timestamp):
        pass
    
    # Register callback
    server.on_scan(1, callback)
    
    # Remove callback
    server.remove_callback(1)
    
    # Should not raise error if removed again
    server.remove_callback(1)


@pytest.mark.asyncio
async def test_lidar_server_thread_pool_execution():
    """Test that callbacks execute in thread pool (non-blocking)."""
    callback_thread_ids = []
    callback_lock = threading.Lock()
    
    def callback(vessel_id, lidar_id, points, timestamp):
        thread_id = threading.current_thread().ident
        with callback_lock:
            callback_thread_ids.append(thread_id)
    
    server = LidarSensorServer(port=18766, max_workers=2)
    server.on_scan(1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points = np.random.randn(100, 4).astype(np.float32)
            
            # Send multiple scans
            for _ in range(5):
                await ws.send(pack_lidar_scan(1, time.time(), points))
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
async def test_lidar_server_stats():
    """Test that server tracks statistics correctly."""
    server = LidarSensorServer(port=18766)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect("ws://localhost:18766") as ws:
            points = np.random.randn(1000, 4).astype(np.float32)
            
            # Send 5 scans
            for _ in range(5):
                await ws.send(pack_lidar_scan(1, time.time(), points))
                await asyncio.sleep(0.05)
            
            # Check stats while connection is still open
            stats = server.get_stats()
            assert stats['messages'] == 5
            assert stats['bytes'] > 0  # Should have received bytes
            assert stats['connections'] == 1  # Connection still open
            assert stats['total_scans'] == 5
            assert stats['scans_per_vessel']['1'] == 5
        
        # Wait for connection cleanup after context manager exits
        await asyncio.sleep(0.2)
        
        # Check stats after connection closed
        stats_after = server.get_stats()
        assert stats_after['messages'] == 5
        assert stats_after['bytes'] > 0
        assert stats_after['connections'] == 0  # Connection closed
        assert stats_after['total_scans'] == 5
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


@pytest.mark.asyncio
async def test_lidar_server_various_point_counts():
    """Test server handles various point counts (1K, 10K, 50K)."""
    received_counts = []
    callback_lock = threading.Lock()
    
    def callback(vessel_id, lidar_id, points, timestamp):
        with callback_lock:
            received_counts.append(points.shape[0])
    
    server = LidarSensorServer(port=18766)
    server.on_scan(1, callback)
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        # Send scans one at a time and wait for each to be processed
        # Using smaller sizes to avoid WebSocket message size limits
        for point_count in [1000, 10000, 50000]:
            async with websockets.connect("ws://localhost:18766", max_size=None) as ws:
                points = np.random.randn(point_count, 4).astype(np.float32)
                scan = pack_lidar_scan(1, time.time(), points)
                await ws.send(scan)
                # Wait for callback to complete before closing connection
                max_wait = 5.0  # Increased wait time for large scans
                waited = 0.0
                initial_count = len(received_counts)
                while len(received_counts) == initial_count and waited < max_wait:
                    await asyncio.sleep(0.2)
                    waited += 0.2
                # Additional delay to ensure processing is complete
                await asyncio.sleep(0.3)
        
        # Final check - all scans should be received
        assert len(received_counts) == 3, f"Expected 3 scans, got {len(received_counts)}: {received_counts}"
        # Verify we got the expected point counts (order may vary due to async processing)
        received_counts_sorted = sorted(received_counts)
        assert received_counts_sorted == [1000, 10000, 50000], f"Expected [1000, 10000, 50000], got {received_counts_sorted}"
    
    finally:
        await server.stop()
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass

