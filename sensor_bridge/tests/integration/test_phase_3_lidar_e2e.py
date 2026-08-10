"""
Phase 3 Integration Test: Lidar End-to-End

Full end-to-end test of lidar point cloud streaming in Docker environment.
Tests the complete flow: mock browser client → bridge → controller callback

Test scenarios:
- Lidar server accepts WebSocket connections
- Point clouds flow: mock client → bridge → controller callback
- Various point counts (1K, 10K, 100K)
- High throughput: 100K points at 10Hz for 5 seconds
- Multiple vessels work simultaneously
- Bridge survives reconnection

Run with:
  pytest sensor_bridge/tests/integration/test_phase_3_lidar_e2e.py -v

Or use: bash sensor_bridge/scripts/test_task_3_6_docker.sh
"""

import asyncio
import os
import pytest
import threading
import time

import numpy as np
import websockets

from mavsim_sensor_bridge.utils.binary import pack_lidar_scan

# Bridge connection settings (configurable via environment for Docker)
BRIDGE_HOST = os.environ.get('BRIDGE_HOST', 'localhost')
LIDAR_PORT = int(os.environ.get('LIDAR_PORT', '8766'))
LIDAR_WS_URL = f"ws://{BRIDGE_HOST}:{LIDAR_PORT}"

# Delay for bridge to be ready
BRIDGE_READY_DELAY = float(os.environ.get('BRIDGE_READY_DELAY', '2.0'))


# ---------------------------------------------------------------------------
# Shared state for tracking received scans
# ---------------------------------------------------------------------------

class ScanTracker:
    """Thread-safe scan tracker for lidar callbacks."""

    def __init__(self):
        self.scans = []
        self.lock = threading.Lock()

    def add_scan(self, vessel_id: int, lidar_id: int, points: np.ndarray, timestamp: float):
        """Record a received scan."""
        with self.lock:
            self.scans.append({
                'point_count': points.shape[0],
                'timestamp': timestamp,
                'dtype': str(points.dtype),
                'shape': points.shape,
            })

    def get_scans(self):
        """Return a copy of all recorded scans."""
        with self.lock:
            return list(self.scans)

    def clear(self):
        """Clear all recorded scans."""
        with self.lock:
            self.scans.clear()

    def count(self):
        """Return the number of recorded scans."""
        with self.lock:
            return len(self.scans)


# Global tracker
scan_tracker = ScanTracker()


# ---------------------------------------------------------------------------
# Helper: start bridge, run body, stop bridge
# ---------------------------------------------------------------------------

async def _run_with_bridge(body, *, lidar_port=LIDAR_PORT, camera_enabled=False):
    """Start a SensorBridge with lidar enabled, run *body*, then stop."""
    from mavsim_sensor_bridge import SensorBridge
    from mavsim_sensor_bridge.config import BridgeConfig

    config = BridgeConfig(
        camera_enabled=camera_enabled,
        lidar_enabled=True,
        lidar_port=lidar_port,
    )
    bridge = SensorBridge(config=config)
    bridge_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.5)  # let server bind

    try:
        await body(bridge)
    finally:
        await bridge.stop()
        if not bridge_task.done():
            bridge_task.cancel()
            try:
                await bridge_task
            except asyncio.CancelledError:
                pass


# ===========================================================================
# Test 1: Lidar server accepts connections
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_server_accepts_connections():
    """Test that the lidar server accepts WebSocket connections."""

    async def body(bridge):
        # Simply open a WebSocket connection to the lidar server
        ws_url = f"ws://localhost:{LIDAR_PORT}"
        async with websockets.connect(ws_url) as ws:
            # Connection succeeded – send a single small scan to confirm
            points = np.random.randn(10, 4).astype(np.float32)
            await ws.send(pack_lidar_scan(1, time.time(), points))
            await asyncio.sleep(0.2)

        # Verify stats show at least 1 message & 1 connection processed
        stats = bridge.get_server_stats('lidar')
        assert stats['messages'] >= 1, f"Expected ≥1 message, got {stats['messages']}"

    await _run_with_bridge(body)


# ===========================================================================
# Test 2: Point clouds flow mock client → bridge → controller callback
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_point_cloud_flow():
    """Test that point clouds sent to the bridge invoke controller callbacks."""
    scan_tracker.clear()

    async def body(bridge):
        bridge.on_lidar(1, scan_tracker.add_scan)

        ws_url = f"ws://localhost:{LIDAR_PORT}"
        async with websockets.connect(ws_url) as ws:
            for i in range(10):
                points = np.random.randn(1000, 4).astype(np.float32)
                await ws.send(pack_lidar_scan(1, time.time(), points))
                await asyncio.sleep(0.033)

        # Wait for all callbacks to execute
        await asyncio.sleep(1.0)

        scans = scan_tracker.get_scans()
        assert len(scans) >= 8, f"Expected ≥8 scans, got {len(scans)}"
        for s in scans:
            assert s['point_count'] == 1000
            assert s['dtype'] == 'float32'
            assert s['shape'] == (1000, 4)

    await _run_with_bridge(body)


# ===========================================================================
# Test 3: Various point counts (1K, 10K, 100K)
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_various_point_counts():
    """Test that the bridge handles various point counts correctly."""
    scan_tracker.clear()

    async def body(bridge):
        bridge.on_lidar(1, scan_tracker.add_scan)

        for point_count in [1000, 10000, 100000]:
            ws_url = f"ws://localhost:{LIDAR_PORT}"
            async with websockets.connect(ws_url, max_size=None) as ws:
                points = np.random.randn(point_count, 4).astype(np.float32)
                await ws.send(pack_lidar_scan(1, time.time(), points))

                # Wait for callback to complete before moving to next size
                max_wait = 5.0
                initial = scan_tracker.count()
                waited = 0.0
                while scan_tracker.count() == initial and waited < max_wait:
                    await asyncio.sleep(0.2)
                    waited += 0.2
                await asyncio.sleep(0.3)

        scans = scan_tracker.get_scans()
        assert len(scans) == 3, f"Expected 3 scans, got {len(scans)}"
        received_counts = sorted(s['point_count'] for s in scans)
        assert received_counts == [1000, 10000, 100000], (
            f"Expected [1000, 10000, 100000], got {received_counts}"
        )

    await _run_with_bridge(body)


# ===========================================================================
# Test 4: High throughput – 100K points at 10 Hz for 5 seconds (50 scans)
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_high_throughput():
    """Test 100K points at 10 Hz for 5 seconds (50 scans total)."""
    scan_tracker.clear()

    async def body(bridge):
        bridge.on_lidar(1, scan_tracker.add_scan)

        ws_url = f"ws://localhost:{LIDAR_PORT}"
        async with websockets.connect(ws_url, max_size=None) as ws:
            points = np.random.randn(100000, 4).astype(np.float32)
            start_time = time.time()
            for i in range(50):
                await ws.send(pack_lidar_scan(1, time.time(), points))
                await asyncio.sleep(0.1)  # 10 Hz
            elapsed = time.time() - start_time

        # Wait for all callbacks to finish (large scans take time)
        await asyncio.sleep(3.0)

        scans = scan_tracker.get_scans()
        # Allow some tolerance — at least 80 % of 50 = 40 scans
        assert len(scans) >= 40, (
            f"Expected ≥40 scans (of 50), got {len(scans)} in {elapsed:.1f}s"
        )
        for s in scans:
            assert s['point_count'] == 100000

    await _run_with_bridge(body)


# ===========================================================================
# Test 5: Multiple vessels streaming simultaneously
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_multiple_vessels():
    """Test that multiple vessels can stream lidar data simultaneously."""
    tracker_v1 = ScanTracker()
    tracker_v2 = ScanTracker()

    async def body(bridge):
        bridge.on_lidar(1, tracker_v1.add_scan)
        bridge.on_lidar(2, tracker_v2.add_scan)

        ws_url = f"ws://localhost:{LIDAR_PORT}"

        async def stream_vessel(vessel_id, num_scans=20, num_points=5000):
            async with websockets.connect(ws_url, max_size=None) as ws:
                for i in range(num_scans):
                    pts = np.random.randn(num_points, 4).astype(np.float32)
                    await ws.send(pack_lidar_scan(vessel_id, time.time(), pts))
                    await asyncio.sleep(0.05)

        await asyncio.gather(
            stream_vessel(1, num_scans=20, num_points=5000),
            stream_vessel(2, num_scans=20, num_points=3000),
        )

        # Wait for callbacks
        await asyncio.sleep(1.0)

        v1 = tracker_v1.get_scans()
        v2 = tracker_v2.get_scans()
        assert len(v1) >= 16, f"Vessel 1: expected ≥16 scans, got {len(v1)}"
        assert len(v2) >= 16, f"Vessel 2: expected ≥16 scans, got {len(v2)}"
        for s in v1:
            assert s['point_count'] == 5000
        for s in v2:
            assert s['point_count'] == 3000

    await _run_with_bridge(body)


# ===========================================================================
# Test 6: Reconnection resilience
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_reconnection():
    """Test that bridge handles client reconnection."""
    scan_tracker.clear()

    async def body(bridge):
        bridge.on_lidar(1, scan_tracker.add_scan)

        ws_url = f"ws://localhost:{LIDAR_PORT}"

        for reconnect_idx in range(3):
            async with websockets.connect(ws_url) as ws:
                points = np.random.randn(500, 4).astype(np.float32)
                await ws.send(pack_lidar_scan(1, time.time(), points))
                await asyncio.sleep(0.1)
            # Disconnect, then reconnect
            await asyncio.sleep(0.2)

        # Wait for callbacks
        await asyncio.sleep(0.5)

        scans = scan_tracker.get_scans()
        assert len(scans) >= 2, (
            f"Expected ≥2 scans across 3 reconnections, got {len(scans)}"
        )

    await _run_with_bridge(body)


# ===========================================================================
# Test 7: Bridge stats after lidar streaming
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_bridge_stats():
    """Test that bridge stats are updated correctly after lidar streaming."""

    async def body(bridge):
        bridge.on_lidar(1, lambda vid, lid, pts, ts: None)

        ws_url = f"ws://localhost:{LIDAR_PORT}"
        async with websockets.connect(ws_url) as ws:
            for i in range(10):
                pts = np.random.randn(200, 4).astype(np.float32)
                await ws.send(pack_lidar_scan(1, time.time(), pts))
                await asyncio.sleep(0.05)

        await asyncio.sleep(0.5)

        stats = bridge.get_server_stats('lidar')
        assert stats['messages'] == 10, f"Expected 10 messages, got {stats['messages']}"
        assert stats['bytes'] > 0
        assert stats['total_scans'] == 10
        # Per-sensor key "<vessel>:<lidar>"; pack_lidar_scan defaults lidar_id=0.
        # Was scans_per_vessel['1'], which get_stats() no longer returns. Not
        # caught earlier because this suite needs a live bridge and is not in CI.
        assert stats['scans_per_sensor']['1:0'] == 10

    await _run_with_bridge(body)


# ===========================================================================
# Test 8: Combined camera + lidar bridge
# ===========================================================================

@pytest.mark.asyncio
async def test_lidar_and_camera_coexist():
    """Test that lidar and camera servers can run simultaneously in the bridge."""
    scan_tracker.clear()

    async def body(bridge):
        bridge.on_lidar(1, scan_tracker.add_scan)
        # We don't send camera data here; we just verify both servers start

        # Verify both servers are present
        stats_all = bridge.get_server_stats()
        assert 'lidar' in stats_all, "Lidar server should be present"
        assert 'camera' in stats_all, "Camera server should be present"

        # Send lidar data and confirm it flows
        ws_url = f"ws://localhost:{LIDAR_PORT}"
        async with websockets.connect(ws_url) as ws:
            pts = np.random.randn(500, 4).astype(np.float32)
            await ws.send(pack_lidar_scan(1, time.time(), pts))
            await asyncio.sleep(0.2)

        assert scan_tracker.count() >= 1

    await _run_with_bridge(body, camera_enabled=True)

