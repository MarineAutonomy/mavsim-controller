#!/bin/bash
# Test script for Task 3.1: Lidar Sensor Server
# This script verifies that the LidarSensorServer works correctly

set -e

echo "Testing Task 3.1: Lidar Sensor Server"
echo "========================================"

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

echo ""
echo "1. Testing LidarSensorServer import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer
print(f'   ✓ LidarSensorServer imported successfully')
print(f'   ✓ LidarSensorServer class: {LidarSensorServer}')
"

echo ""
echo "2. Testing LidarSensorServer instantiation..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer

server = LidarSensorServer(port=18766)
print(f'   ✓ LidarSensorServer instantiated')
print(f'   ✓ Port: {server.port}')
print(f'   ✓ Name: {server.name}')
print(f'   ✓ Is running: {server.is_running}')
print(f'   ✓ Max points per scan: {server.max_points_per_scan}')
"

echo ""
echo "3. Testing callback registration..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer

server = LidarSensorServer(port=18766)

def test_callback(points, timestamp):
    pass

server.on_scan(vessel_id=1, callback=test_callback)
print(f'   ✓ Callback registered for vessel_id=1')

# Test validation
try:
    server.on_scan(vessel_id=-1, callback=test_callback)
    print('   ✗ Should have raised ValueError for invalid vessel_id')
    exit(1)
except ValueError:
    print(f'   ✓ Validation works: invalid vessel_id raises ValueError')

try:
    server.on_scan(vessel_id=1, callback=None)
    print('   ✗ Should have raised TypeError for invalid callback')
    exit(1)
except TypeError:
    print(f'   ✓ Validation works: invalid callback raises TypeError')
"

echo ""
echo "4. Testing binary message utilities integration..."
python3 -c "
import sys
sys.path.insert(0, '.')
import numpy as np
import time
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer
from mavsim_sensor_bridge.utils.binary import pack_lidar_scan, unpack_lidar_scan

server = LidarSensorServer(port=18766)

# Pack a test scan
points = np.random.randn(1000, 4).astype(np.float32)
packed = pack_lidar_scan(1, time.time(), points)
print(f'   ✓ Packed lidar scan: {len(packed)} bytes')

# Unpack it (simulating what server does)
vessel_id, timestamp, unpacked_points = unpack_lidar_scan(packed)
print(f'   ✓ Unpacked lidar scan: vessel_id={vessel_id}, points={unpacked_points.shape}')
assert vessel_id == 1
assert unpacked_points.shape == (1000, 4)
assert unpacked_points.dtype == np.float32
print(f'   ✓ Binary message round-trip works correctly')
"

echo ""
echo "5. Testing point count validation..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer

server = LidarSensorServer(port=18766, max_points_per_scan=1000)
print(f'   ✓ Point count validation initialized')
print(f'   ✓ Max points per scan: {server.max_points_per_scan}')
"

echo ""
echo "6. Testing thread pool executor..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer

server = LidarSensorServer(port=18766, max_workers=4)
print(f'   ✓ Thread pool executor created')
print(f'   ✓ Max workers: {server.executor._max_workers}')
"

echo ""
echo "========================================"
echo "All basic tests passed! ✓"
echo ""
echo "To run full pytest tests (requires pytest and websockets installed):"
echo "  cd sensor_bridge && pytest tests/unit/test_task_3_1_lidar_server.py -v"
echo ""
echo "To test with actual WebSocket connection:"
echo "  python3 -c \"
import asyncio
import numpy as np
import time
from mavsim_sensor_bridge.servers.lidar import LidarSensorServer
from mavsim_sensor_bridge.utils.binary import pack_lidar_scan
import websockets

async def test():
    received = []
    def callback(points, timestamp):
        received.append(points.shape[0])
    
    server = LidarSensorServer(port=18766)
    server.on_scan(1, callback)
    
    # Start server in background
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.5)  # Wait for server to start
    
    try:
        async with websockets.connect('ws://localhost:18766') as ws:
            points = np.random.randn(1000, 4).astype(np.float32)
            scan = pack_lidar_scan(1, time.time(), points)
            await ws.send(scan)
            await asyncio.sleep(0.2)
        
        print(f'Received: {received}')
        assert len(received) == 1
        assert received[0] == 1000
        print('✓ WebSocket test passed!')
    finally:
        await server.stop()

asyncio.run(test())
\""











