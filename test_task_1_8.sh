#!/bin/bash
# Test script for Task 1.8: Bridge Configuration
# This script verifies that BridgeConfig works correctly (defaults, YAML, env)

set -e

echo "Testing Task 1.8: Bridge Configuration"
echo "========================================"

# Change to sensor_bridge directory
cd "$(dirname "$0")"

echo ""
echo "1. Testing config module import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.config import BridgeConfig, DEFAULT_CAMERA_PORT, DEFAULT_LIDAR_PORT
print(f'   ✓ Config module imported successfully')
print(f'   ✓ BridgeConfig: {BridgeConfig}')
print(f'   ✓ Default camera port constant: {DEFAULT_CAMERA_PORT}')
print(f'   ✓ Default lidar port constant: {DEFAULT_LIDAR_PORT}')
"

echo ""
echo "2. Testing default config..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.config import BridgeConfig

config = BridgeConfig()
assert config.camera_port == 8765, f'Expected camera_port=8765, got {config.camera_port}'
assert config.lidar_port == 8766, f'Expected lidar_port=8766, got {config.lidar_port}'
assert config.sonar_port == 8767, f'Expected sonar_port=8767, got {config.sonar_port}'
assert config.depth_port == 8768, f'Expected depth_port=8768, got {config.depth_port}'
assert config.auxiliary_port == 8769, f'Expected auxiliary_port=8769, got {config.auxiliary_port}'
assert config.camera_enabled == True, f'Expected camera_enabled=True, got {config.camera_enabled}'
print(f'   ✓ Default config: camera_port={config.camera_port}, lidar_port={config.lidar_port}')
print(f'   ✓ All default ports and options correct')
"

echo ""
echo "3. Testing config from YAML..."
CONFIG_FILE="$(pwd)/_test_config_task_1_8.yaml"
echo 'camera_port: 9000' > "$CONFIG_FILE"
export CONFIG_FILE
python3 -c "
import sys
import os
sys.path.insert(0, '.')
from mavsim_sensor_bridge.config import BridgeConfig
yaml_path = os.environ.get('CONFIG_FILE', '')
assert yaml_path and os.path.exists(yaml_path), 'Config file not found'
config = BridgeConfig.from_yaml(yaml_path)
assert config.camera_port == 9000, f'Expected camera_port=9000, got {config.camera_port}'
print(f'   ✓ Config from YAML: camera_port={config.camera_port}')
print(f'   ✓ from_yaml() loads camera_port=9000 correctly')
"
rm -f "$CONFIG_FILE"
unset CONFIG_FILE

echo ""
echo "4. Testing config from environment..."
python3 -c "
import sys
import os
sys.path.insert(0, '.')
from mavsim_sensor_bridge.config import BridgeConfig

# Set env and load
os.environ['SENSOR_BRIDGE_CAMERA_PORT'] = '9001'
config = BridgeConfig.from_env()
assert config.camera_port == 9001, f'Expected camera_port=9001, got {config.camera_port}'
print(f'   ✓ Config from env: SENSOR_BRIDGE_CAMERA_PORT=9001 -> camera_port={config.camera_port}')
# Clean up for other tests
del os.environ['SENSOR_BRIDGE_CAMERA_PORT']
"

echo ""
echo "5. Testing BridgeConfig with SensorBridge..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig

config = BridgeConfig(camera_port=9100, camera_enabled=True)
bridge = SensorBridge(config=config)
assert bridge.config.camera_port == 9100, f'Expected 9100, got {bridge.config.camera_port}'
assert bridge.config.lidar_port == 8766, f'Expected default lidar_port 8766, got {bridge.config.lidar_port}'
print(f'   ✓ SensorBridge accepts BridgeConfig: camera_port={bridge.config.camera_port}')
"

echo ""
echo "6. Testing package export..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge import BridgeConfig
config = BridgeConfig()
assert config.camera_port == 8765
print(f'   ✓ BridgeConfig importable from mavsim_sensor_bridge package')
"

echo ""
echo "7. Running pytest tests..."
if command -v pytest &> /dev/null; then
    if timeout 60 pytest tests/unit/test_task_1_8_config.py -v --tb=short; then
        echo "   ✓ All pytest tests passed"
    else
        echo "   ✗ Some pytest tests failed or timed out"
        exit 1
    fi
else
    echo "   ⚠ pytest not found, skipping pytest tests"
    echo "   Install pytest with: pip install pytest pytest-asyncio"
    echo "   Note: Basic functionality tests above all passed ✓"
fi

echo ""
echo "========================================"
echo "All tests passed! ✓"
echo ""
echo "To run pytest tests manually:"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_8_config.py -v"
echo ""
echo "To test config manually:"
echo "  python3 -c \"from mavsim_sensor_bridge.config import BridgeConfig; c = BridgeConfig.from_env(); print(c)\""
echo "  SENSOR_BRIDGE_CAMERA_PORT=9999 python3 -c \"from mavsim_sensor_bridge.config import BridgeConfig; c = BridgeConfig.from_env(); print(c.camera_port)\""
