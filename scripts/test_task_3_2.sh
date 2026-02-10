#!/bin/bash
# Test script for Task 3.2: Update SensorBridge for Lidar
# Verifies that SensorBridge includes lidar server and on_lidar() works.

set -e

echo "Testing Task 3.2: Update SensorBridge for Lidar"
echo "================================================"

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

echo ""
echo "1. Testing SensorBridge with lidar server..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge
from mavsim_sensor_bridge.config import BridgeConfig

# Lidar enabled
config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=18767)
bridge = SensorBridge(config=config)
assert 'lidar' in bridge._servers, 'Bridge should have lidar server when lidar_enabled=True'
assert bridge._servers['lidar'].port == 18767
print('   ✓ SensorBridge includes lidar server when lidar_enabled=True')

# Lidar disabled
config_off = BridgeConfig(lidar_enabled=False)
bridge_off = SensorBridge(config=config_off)
assert 'lidar' not in bridge_off._servers, 'Bridge should not have lidar server when lidar_enabled=False'
print('   ✓ SensorBridge omits lidar server when lidar_enabled=False')
"

echo ""
echo "2. Testing on_lidar registration..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge
from mavsim_sensor_bridge.config import BridgeConfig

config = BridgeConfig(camera_enabled=False, lidar_enabled=True, lidar_port=18767)
bridge = SensorBridge(config=config)

def dummy_callback(points, timestamp):
    pass

bridge.on_lidar(1, dummy_callback)
print('   ✓ on_lidar(1, callback) registered')

# When lidar disabled, on_lidar should raise
bridge_no_lidar = SensorBridge(config=BridgeConfig(lidar_enabled=False))
try:
    bridge_no_lidar.on_lidar(1, dummy_callback)
    print('   ✗ Expected ValueError when lidar not enabled')
    sys.exit(1)
except ValueError as e:
    if 'not enabled' in str(e):
        print('   ✓ on_lidar raises ValueError when lidar server not enabled')
    else:
        raise
"

echo ""
echo "3. Testing config lidar_port and lidar_enabled..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.config import BridgeConfig

c = BridgeConfig()
assert hasattr(c, 'lidar_port')
assert c.lidar_port == 8766
assert hasattr(c, 'lidar_enabled')
assert c.lidar_enabled is True
print('   ✓ Default config: lidar_port=8766, lidar_enabled=True')

c2 = BridgeConfig(lidar_port=9000, lidar_enabled=False)
assert c2.lidar_port == 9000
assert c2.lidar_enabled is False
print('   ✓ Custom config: lidar_port and lidar_enabled respected')
"

echo ""
echo "================================================"
echo "All basic tests passed! ✓"
echo ""
echo "To run full pytest unit tests:"
echo "  cd sensor_bridge && pytest tests/unit/test_task_3_2_bridge_lidar.py -v"
echo ""
echo "To run dockerized test:"
echo "  ./scripts/test_task_3_2_docker.sh"
echo ""
