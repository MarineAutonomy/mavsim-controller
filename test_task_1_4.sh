#!/bin/bash
# Test script for Task 1.4: Camera Sensor Server
# This script verifies that the CameraSensorServer works correctly

set -e

echo "Testing Task 1.4: Camera Sensor Server"
echo "========================================"

# Change to sensor_bridge directory
cd "$(dirname "$0")"

echo ""
echo "1. Testing CameraSensorServer import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.camera import CameraSensorServer
print(f'   ✓ CameraSensorServer imported successfully')
print(f'   ✓ CameraSensorServer class: {CameraSensorServer}')
"

echo ""
echo "2. Testing CameraSensorServer instantiation..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.camera import CameraSensorServer

server = CameraSensorServer(port=18765)
print(f'   ✓ CameraSensorServer instantiated')
print(f'   ✓ Port: {server.port}')
print(f'   ✓ Name: {server.name}')
print(f'   ✓ Is running: {server.is_running}')
"

echo ""
echo "3. Testing callback registration..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.camera import CameraSensorServer

server = CameraSensorServer(port=18765)

def test_callback(vessel_id, camera_id, timestamp, jpeg_data):
    pass

server.on_frame(vessel_id=1, camera_id=1, callback=test_callback)
print(f'   ✓ Callback registered for vessel_id=1, camera_id=1')

# Test validation
try:
    server.on_frame(vessel_id=-1, camera_id=1, callback=test_callback)
    print('   ✗ Should have raised ValueError for invalid vessel_id')
    exit(1)
except ValueError:
    print(f'   ✓ Validation works: invalid vessel_id raises ValueError')

try:
    server.on_frame(vessel_id=1, camera_id=256, callback=test_callback)
    print('   ✗ Should have raised ValueError for invalid camera_id')
    exit(1)
except ValueError:
    print(f'   ✓ Validation works: invalid camera_id raises ValueError')
"

echo ""
echo "4. Testing binary message utilities integration..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.camera import CameraSensorServer
from mavsim_sensor_bridge.utils.binary import pack_camera_frame, unpack_camera_frame
import time

server = CameraSensorServer(port=18765)

# Pack a test frame
jpeg_data = b'\xff\xd8\xff\xe0\x00\x10JFIF'
packed = pack_camera_frame(1, 1, time.time(), jpeg_data)
print(f'   ✓ Packed camera frame: {len(packed)} bytes')

# Unpack it (simulating what server does)
vessel_id, camera_id, timestamp, unpacked_jpeg = unpack_camera_frame(packed)
print(f'   ✓ Unpacked camera frame: vessel_id={vessel_id}, camera_id={camera_id}')
assert vessel_id == 1
assert camera_id == 1
assert unpacked_jpeg == jpeg_data
print(f'   ✓ Binary message round-trip works correctly')
"

echo ""
echo "5. Testing frame drop detection..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.camera import CameraSensorServer

server = CameraSensorServer(port=18765)

def dummy_callback(vessel_id, camera_id, timestamp, jpeg_data):
    pass

server.on_frame(1, 1, dummy_callback)

# Simulate frame drops by setting timestamps with gaps
with server._sequence_lock:
    server.last_timestamps[(1, 1)] = 1000.0
    # Simulate large gap
    server.last_timestamps[(1, 1)] = 2000.0  # 1 second gap

dropped = server.get_dropped_frames(1, 1)
print(f'   ✓ Frame drop detection initialized')
print(f'   ✓ Dropped frames counter: {dropped}')

server.reset_dropped_frames(1, 1)
dropped_after_reset = server.get_dropped_frames(1, 1)
assert dropped_after_reset == 0
print(f'   ✓ Reset dropped frames works')
"

echo ""
echo "6. Testing thread pool executor..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.camera import CameraSensorServer

server = CameraSensorServer(port=18765, max_workers=4)
print(f'   ✓ Thread pool executor created')
print(f'   ✓ Max workers: {server.executor._max_workers}')
"

echo ""
echo "========================================"
echo "All basic tests passed! ✓"
echo ""
echo "To run full pytest tests (requires pytest and websockets installed):"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_4_camera_server.py -v"
echo ""
echo "To test with actual WebSocket connection:"
echo "  python3 -c \"
import asyncio
from mavsim_sensor_bridge.servers.camera import CameraSensorServer
from mavsim_sensor_bridge.utils.binary import pack_camera_frame
import websockets
import time

async def test():
    received = []
    def callback(v, c, t, j):
        received.append((v, c))
    
    server = CameraSensorServer(port=18765)
    server.on_frame(1, 1, callback)
    
    # Start server in background
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect('ws://localhost:18765') as ws:
            frame = pack_camera_frame(1, 1, time.time(), b'\\xff\\xd8\\xff')
            await ws.send(frame)
            await asyncio.sleep(0.2)
        
        print(f'Received: {received}')
        assert len(received) == 1
        print('✓ WebSocket test passed!')
    finally:
        await server.stop()

asyncio.run(test())
\""
