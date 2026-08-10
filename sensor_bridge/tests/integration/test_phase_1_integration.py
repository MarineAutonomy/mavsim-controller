"""
Phase 1 Integration Test

End-to-end test verifying Phase 1 works in Docker environment.

Test scenarios:
- Bridge starts in Docker container
- External client connects to camera server
- Camera frames received (callbacks invoked on bridge side; test verifies send path)
- Multiple concurrent connections work
- Graceful connection close works

Run with:
  docker-compose -f docker-compose.test.yml run test pytest tests/integration/test_phase_1_integration.py -v

Or use: bash test_task_1_10_docker.sh
"""

import asyncio
import os
import pytest
import websockets
from mavsim_sensor_bridge.utils.binary import pack_camera_frame

BRIDGE_HOST = os.environ.get("BRIDGE_HOST", "localhost")
BRIDGE_PORT = 8765
WS_URL = f"ws://{BRIDGE_HOST}:{BRIDGE_PORT}"

# Short delay for bridge to be ready in Docker
BRIDGE_READY_DELAY = 0.5


@pytest.mark.asyncio
async def test_camera_server_end_to_end():
    """Test camera frame flow: client → bridge → (callback on bridge; no exception = pass)."""
    await asyncio.sleep(BRIDGE_READY_DELAY)

    async with websockets.connect(WS_URL) as ws:
        for i in range(100):
            frame = pack_camera_frame(
                vessel_id=1,
                camera_id=1,
                timestamp=1706400000.0 + i * 0.033,
                jpeg_data=b"\xff\xd8\xff\xe0" + b"\x00" * 10000,
            )
            await ws.send(frame)
            await asyncio.sleep(0.01)

    # If no exception, frame path works (verify via stats endpoint if added later)


@pytest.mark.asyncio
async def test_multiple_cameras():
    """Test multiple camera connections simultaneously."""
    await asyncio.sleep(BRIDGE_READY_DELAY)

    async def send_frames(vessel_id: int, camera_id: int, count: int) -> None:
        async with websockets.connect(WS_URL) as ws:
            for i in range(count):
                frame = pack_camera_frame(
                    vessel_id, camera_id, 1706400000.0 + i, b"\xff\xd8\xff\xe0" + b"\x00" * 4995
                )
                await ws.send(frame)
                await asyncio.sleep(0.01)

    await asyncio.gather(
        send_frames(1, 1, 50),
        send_frames(1, 2, 50),
        send_frames(2, 1, 50),
        send_frames(2, 2, 50),
    )


@pytest.mark.asyncio
async def test_high_throughput():
    """Test multiple cameras at 30 Hz for 2 seconds (quick Phase 1 throughput check)."""
    await asyncio.sleep(BRIDGE_READY_DELAY)

    async def camera_stream(
        vessel_id: int, camera_id: int, duration_sec: float, fps: float
    ) -> None:
        async with websockets.connect(WS_URL) as ws:
            frames = int(duration_sec * fps)
            for i in range(frames):
                frame = pack_camera_frame(
                    vessel_id,
                    camera_id,
                    1706400000.0 + i / fps,
                    b"\xff\xd8\xff\xe0" + b"\x00" * 49995,
                )
                await ws.send(frame)
                await asyncio.sleep(1.0 / fps)

    # 4 cameras at 30 Hz for 2 seconds (quick test; full would be 20 cameras for 5 sec)
    tasks = [
        camera_stream(v, c, duration_sec=2.0, fps=30.0)
        for v in range(1, 3)
        for c in range(1, 3)
    ]
    await asyncio.gather(*tasks)


@pytest.mark.asyncio
async def test_graceful_connection_close():
    """Test that closing the WebSocket cleanly (CLOSE frame) is handled by the bridge."""
    await asyncio.sleep(BRIDGE_READY_DELAY)

    async with websockets.connect(WS_URL) as ws:
        for i in range(5):
            frame = pack_camera_frame(
                1, 1, 1706400000.0 + i, b"\xff\xd8\xff\xe0" + b"\x00" * 100
            )
            await ws.send(frame)
            await asyncio.sleep(0.02)
    # Exit context manager sends WebSocket CLOSE; if no exception, graceful close works
