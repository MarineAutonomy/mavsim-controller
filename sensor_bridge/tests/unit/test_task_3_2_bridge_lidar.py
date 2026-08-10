"""
Unit tests for Task 3.2: Update SensorBridge for Lidar

Tests verify that the SensorBridge:
- Adds LidarSensorServer when lidar_enabled is True
- Implements on_lidar(vessel_id, callback) and forwards to lidar server
- Bridge handles lidar callbacks when receiving binary lidar scans over WebSocket
- Config includes lidar port and lidar_enabled
- Raises ValueError when on_lidar is called but lidar server is not enabled
"""

import asyncio
import time
import numpy as np
import pytest
import websockets

from mavsim_sensor_bridge.bridge import SensorBridge
from mavsim_sensor_bridge.config import BridgeConfig
from mavsim_sensor_bridge.utils.binary import pack_lidar_scan


# Use non-default lidar port to avoid conflicts with other tests (e.g. task 3.1 uses 18766)
LIDAR_TEST_PORT = 18767


@pytest.mark.asyncio
async def test_bridge_lidar():
    """Test that bridge receives lidar scans and invokes callbacks (plan verification)."""
    received = []
    config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=LIDAR_TEST_PORT)
    bridge = SensorBridge(config=config)
    bridge.on_lidar(1, lambda vid, lid, points, ts: received.append(points))

    # Start bridge in background (only lidar server will run)
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.5)

    try:
        async with websockets.connect(f"ws://localhost:{LIDAR_TEST_PORT}") as ws:
            points = np.random.randn(500, 4).astype(np.float32)
            await ws.send(pack_lidar_scan(1, time.time(), points))
            await asyncio.sleep(0.2)

        assert len(received) == 1
        assert received[0].shape == (500, 4)
        assert received[0].dtype == np.float32
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


@pytest.mark.asyncio
async def test_bridge_lidar_multiple_scans():
    """Test that bridge handles multiple lidar scans for the same vessel."""
    received = []
    config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=LIDAR_TEST_PORT)
    bridge = SensorBridge(config=config)
    bridge.on_lidar(1, lambda vid, lid, points, ts: received.append(points.shape[0]))

    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.5)

    try:
        async with websockets.connect(f"ws://localhost:{LIDAR_TEST_PORT}") as ws:
            for n in [100, 200, 500]:
                points = np.random.randn(n, 4).astype(np.float32)
                await ws.send(pack_lidar_scan(1, time.time(), points))
                await asyncio.sleep(0.1)

        await asyncio.sleep(0.3)
        assert len(received) == 3
        assert sorted(received) == [100, 200, 500]
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


@pytest.mark.asyncio
async def test_bridge_lidar_multiple_vessels():
    """Test that bridge routes lidar scans to the correct vessel callback."""
    received_v1 = []
    received_v2 = []
    config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=LIDAR_TEST_PORT)
    bridge = SensorBridge(config=config)
    bridge.on_lidar(1, lambda vid, lid, points, ts: received_v1.append(points.shape[0]))
    bridge.on_lidar(2, lambda vid, lid, points, ts: received_v2.append(points.shape[0]))

    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.5)

    try:
        async with websockets.connect(f"ws://localhost:{LIDAR_TEST_PORT}") as ws:
            await ws.send(pack_lidar_scan(1, time.time(), np.random.randn(100, 4).astype(np.float32)))
            await ws.send(pack_lidar_scan(2, time.time(), np.random.randn(200, 4).astype(np.float32)))
            await asyncio.sleep(0.2)

        assert len(received_v1) == 1 and received_v1[0] == 100
        assert len(received_v2) == 1 and received_v2[0] == 200
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass


def test_bridge_on_lidar_requires_lidar_enabled():
    """Test that on_lidar raises ValueError when lidar server is not enabled."""
    config = BridgeConfig(camera_enabled=True, lidar_enabled=False)
    bridge = SensorBridge(config=config)

    with pytest.raises(ValueError, match="Lidar server is not enabled"):
        bridge.on_lidar(1, lambda vid, lid, points, ts: None)


def test_bridge_config_includes_lidar_port():
    """Test that BridgeConfig includes lidar_port (and lidar_enabled)."""
    config = BridgeConfig()
    assert hasattr(config, "lidar_port")
    assert config.lidar_port == 8766
    assert hasattr(config, "lidar_enabled")
    assert config.lidar_enabled is True

    custom = BridgeConfig(lidar_port=9000, lidar_enabled=False)
    assert custom.lidar_port == 9000
    assert custom.lidar_enabled is False


def test_bridge_includes_lidar_server_when_enabled():
    """Test that SensorBridge creates lidar server when lidar_enabled=True."""
    config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=LIDAR_TEST_PORT)
    bridge = SensorBridge(config=config)
    assert "lidar" in bridge._servers
    assert bridge._servers["lidar"].port == LIDAR_TEST_PORT


def test_bridge_no_lidar_server_when_disabled():
    """Test that SensorBridge does not create lidar server when lidar_enabled=False."""
    config = BridgeConfig(camera_enabled=False, lidar_enabled=False)
    bridge = SensorBridge(config=config)
    assert "lidar" not in bridge._servers


@pytest.mark.asyncio
async def test_bridge_lidar_get_stats():
    """Test that get_server_stats includes lidar when lidar is enabled."""
    config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=LIDAR_TEST_PORT)
    bridge = SensorBridge(config=config)
    bridge.on_lidar(1, lambda vid, lid, points, ts: None)

    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.5)

    try:
        stats_all = bridge.get_server_stats()
        assert "lidar" in stats_all
        assert "total_scans" in stats_all["lidar"]

        stats_lidar = bridge.get_server_stats("lidar")
        assert "messages" in stats_lidar
        assert "total_scans" in stats_lidar
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass
