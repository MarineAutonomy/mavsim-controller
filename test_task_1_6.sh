#!/bin/bash
# Test script for Task 1.6: Main SensorBridge Class
# This script verifies that the SensorBridge class works correctly

set -e

echo "Testing Task 1.6: Main SensorBridge Class"
echo "=========================================="

# Change to sensor_bridge directory
cd "$(dirname "$0")"

echo ""
echo "1. Testing SensorBridge import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig
print(f'   ✓ SensorBridge imported successfully')
print(f'   ✓ BridgeConfig imported successfully')
print(f'   ✓ SensorBridge class: {SensorBridge}')
print(f'   ✓ BridgeConfig class: {BridgeConfig}')
"

echo ""
echo "2. Testing SensorBridge instantiation..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig

bridge = SensorBridge()
print(f'   ✓ SensorBridge instantiated')
print(f'   ✓ Config: camera_port={bridge.config.camera_port}, camera_enabled={bridge.config.camera_enabled}')
print(f'   ✓ Is running: {bridge.is_running}')
print(f'   ✓ Camera server configured: {\"camera\" in bridge._servers}')
"

echo ""
echo "3. Testing custom configuration..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig

config = BridgeConfig(camera_port=9000, camera_enabled=True)
bridge = SensorBridge(config=config)
print(f'   ✓ Custom config applied: camera_port={bridge.config.camera_port}')
assert bridge.config.camera_port == 9000, 'Port should be 9000'
print(f'   ✓ Custom configuration works')
"

echo ""
echo "4. Testing callback registration..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig

bridge = SensorBridge()

def test_callback(vessel_id, camera_id, timestamp, jpeg_data):
    pass

bridge.on_camera(vessel_id=1, camera_id=1, callback=test_callback)
print(f'   ✓ Camera callback registered for vessel_id=1, camera_id=1')

# Test validation - camera server disabled
config_disabled = BridgeConfig(camera_enabled=False)
bridge_disabled = SensorBridge(config=config_disabled)
try:
    bridge_disabled.on_camera(vessel_id=1, camera_id=1, callback=test_callback)
    print('   ✗ Should have raised ValueError for disabled camera server')
    exit(1)
except ValueError as e:
    if 'not enabled' in str(e):
        print(f'   ✓ Validation works: disabled camera server raises ValueError')
    else:
        raise
"

echo ""
echo "5. Testing server statistics..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge

bridge = SensorBridge()

# Get stats before starting
stats = bridge.get_server_stats()
assert 'camera' in stats, 'Should have camera stats'
assert 'messages' in stats['camera'], 'Should have messages key'
assert 'bytes' in stats['camera'], 'Should have bytes key'
assert 'connections' in stats['camera'], 'Should have connections key'

# Get stats for specific sensor
camera_stats = bridge.get_server_stats('camera')
assert 'messages' in camera_stats, 'Should have messages key'

# Test invalid sensor type
try:
    bridge.get_server_stats('nonexistent')
    print('   ✗ Should have raised ValueError for invalid sensor type')
    exit(1)
except ValueError:
    print(f'   ✓ Validation works: invalid sensor type raises ValueError')

print('   ✓ Server statistics work correctly')
"

echo ""
echo "6. Testing bridge start/stop (basic)..."
python3 -c "
import sys
import asyncio
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge

async def test():
    bridge = SensorBridge()
    assert not bridge.is_running, 'Should not be running initially'
    
    # Start bridge in background
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.3)  # Wait for server to start
    
    assert bridge.is_running, 'Should be running after start'
    
    # Stop bridge
    await bridge.stop()
    await asyncio.sleep(0.1)
    
    assert not bridge.is_running, 'Should not be running after stop'
    
    # Clean up
    if not start_task.done():
        start_task.cancel()
        try:
            await start_task
        except asyncio.CancelledError:
            pass
    
    print('   ✓ Bridge start/stop works correctly')

asyncio.run(test())
"

echo ""
echo "7. Testing bridge receives camera frames..."
python3 -c "
import sys
import asyncio
import time
import websockets
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge
from mavsim_sensor_bridge.utils.binary import pack_camera_frame

async def test():
    received = []
    
    def callback(vessel_id, camera_id, timestamp, jpeg_data):
        received.append((vessel_id, camera_id))
    
    bridge = SensorBridge()
    bridge.on_camera(1, 1, callback)
    
    # Start bridge
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.4)  # Wait for server to start
    
    try:
        # Connect and send frame
        async with websockets.connect('ws://localhost:8765') as ws:
            frame = pack_camera_frame(1, 1, time.time(), b'\\xff' * 1000)
            await ws.send(frame)
            await asyncio.sleep(0.3)  # Wait for callback
        
        assert len(received) == 1, f'Should have received 1 frame, got {len(received)}'
        assert received[0] == (1, 1), f'Should have received vessel_id=1, camera_id=1'
        print('   ✓ Bridge receives camera frames and invokes callbacks')
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()
            try:
                await start_task
            except asyncio.CancelledError:
                pass

asyncio.run(test())
"

echo ""
echo "8. Running pytest tests..."
if command -v pytest &> /dev/null; then
    if timeout 60 pytest tests/unit/test_task_1_6_bridge.py -v --tb=short; then
        echo "   ✓ All pytest tests passed"
    else
        echo "   ✗ Some pytest tests failed or timed out"
        exit 1
    fi
else
    echo "   ⚠ pytest not found, skipping pytest tests"
    echo "   Install pytest with: pip install pytest pytest-asyncio websockets"
    echo "   Note: Basic functionality tests above all passed ✓"
fi

echo ""
echo "=========================================="
echo "All tests passed! ✓"
echo ""
echo "To run pytest tests manually:"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_6_bridge.py -v"
echo ""
echo "To test with actual WebSocket connection:"
echo "  python3 -c \"
import asyncio
from mavsim_sensor_bridge.bridge import SensorBridge
from mavsim_sensor_bridge.utils.binary import pack_camera_frame
import websockets
import time

async def test():
    received = []
    def callback(v, c, t, j):
        received.append((v, c))
        print(f'Received frame: vessel_id={v}, camera_id={c}')
    
    bridge = SensorBridge()
    bridge.on_camera(1, 1, callback)
    
    start_task = asyncio.create_task(bridge.start())
    await asyncio.sleep(0.5)
    
    try:
        async with websockets.connect('ws://localhost:8765') as ws:
            for i in range(3):
                frame = pack_camera_frame(1, 1, time.time(), b'\\xff' * 1000)
                await ws.send(frame)
                await asyncio.sleep(0.1)
            await asyncio.sleep(0.3)
        
        print(f'Total frames received: {len(received)}')
    finally:
        await bridge.stop()
        if not start_task.done():
            start_task.cancel()

asyncio.run(test())
\""
