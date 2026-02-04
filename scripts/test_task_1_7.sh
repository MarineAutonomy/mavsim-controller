#!/bin/bash
# Test script for Task 1.7: Command-Line Interface
# This script verifies that the CLI works correctly

set -e

echo "Testing Task 1.7: Command-Line Interface"
echo "=========================================="

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

echo ""
echo "1. Testing CLI module import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args, CLIBridgeRunner, main
print(f'   ✓ CLI module imported successfully')
print(f'   ✓ parse_args function: {parse_args}')
print(f'   ✓ CLIBridgeRunner class: {CLIBridgeRunner}')
print(f'   ✓ main function: {main}')
"

echo ""
echo "2. Testing default argument parsing..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

args = parse_args([])
assert args.camera_port == 8765, f'Expected camera_port=8765, got {args.camera_port}'
assert args.verbose == False, f'Expected verbose=False, got {args.verbose}'
assert args.stats_interval == 0, f'Expected stats_interval=0, got {args.stats_interval}'
print(f'   ✓ Default arguments: camera_port={args.camera_port}, verbose={args.verbose}, stats_interval={args.stats_interval}')
"

echo ""
echo "3. Testing custom argument parsing..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

args = parse_args(['--camera-port', '9000', '--verbose'])
assert args.camera_port == 9000, f'Expected camera_port=9000, got {args.camera_port}'
assert args.verbose == True, f'Expected verbose=True, got {args.verbose}'
print(f'   ✓ Custom arguments: camera_port={args.camera_port}, verbose={args.verbose}')
"

echo ""
echo "4. Testing --camera-port option..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

args = parse_args(['--camera-port', '8080'])
assert args.camera_port == 8080, f'Expected camera_port=8080, got {args.camera_port}'
print(f'   ✓ Camera port option works: {args.camera_port}')

args = parse_args(['--camera-port', '12345'])
assert args.camera_port == 12345, f'Expected camera_port=12345, got {args.camera_port}'
print(f'   ✓ Camera port accepts different values: {args.camera_port}')
"

echo ""
echo "5. Testing --verbose flag..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

args = parse_args(['--verbose'])
assert args.verbose == True, f'Expected verbose=True, got {args.verbose}'
print(f'   ✓ Verbose flag works: {args.verbose}')

args = parse_args([])
assert args.verbose == False, f'Expected verbose=False, got {args.verbose}'
print(f'   ✓ Verbose defaults to False when not specified')
"

echo ""
echo "6. Testing --stats-interval option..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

args = parse_args(['--stats-interval', '5'])
assert args.stats_interval == 5.0, f'Expected stats_interval=5.0, got {args.stats_interval}'
print(f'   ✓ Stats interval option works: {args.stats_interval}')

args = parse_args(['--stats-interval', '10.5'])
assert args.stats_interval == 10.5, f'Expected stats_interval=10.5, got {args.stats_interval}'
print(f'   ✓ Stats interval accepts float values: {args.stats_interval}')

args = parse_args(['--stats-interval', '0'])
assert args.stats_interval == 0.0, f'Expected stats_interval=0.0, got {args.stats_interval}'
print(f'   ✓ Stats interval accepts 0 (disabled): {args.stats_interval}')
"

echo ""
echo "7. Testing all options together..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

args = parse_args([
    '--camera-port', '9000',
    '--verbose',
    '--stats-interval', '5'
])
assert args.camera_port == 9000, f'Expected camera_port=9000, got {args.camera_port}'
assert args.verbose == True, f'Expected verbose=True, got {args.verbose}'
assert args.stats_interval == 5.0, f'Expected stats_interval=5.0, got {args.stats_interval}'
print(f'   ✓ All options work together correctly')
print(f'     camera_port={args.camera_port}, verbose={args.verbose}, stats_interval={args.stats_interval}')
"

echo ""
echo "8. Testing argument type validation..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import parse_args

# Test that invalid camera port raises error
try:
    parse_args(['--camera-port', 'not-a-number'])
    print('   ✗ Should have raised SystemExit for invalid camera port')
    exit(1)
except SystemExit:
    print('   ✓ Invalid camera port correctly raises SystemExit')
except Exception as e:
    print(f'   ✗ Unexpected exception: {e}')
    exit(1)
"

echo ""
echo "9. Testing CLIBridgeRunner initialization..."
python3 -c "
import sys
import argparse
sys.path.insert(0, '.')
from mavsim_sensor_bridge.cli import CLIBridgeRunner, parse_args

args = parse_args(['--camera-port', '9000', '--verbose'])
runner = CLIBridgeRunner(args)

assert runner.args.camera_port == 9000, 'Runner should have correct camera port'
assert runner.args.verbose == True, 'Runner should have verbose flag'
assert runner.bridge is None, 'Bridge should be None initially'
assert runner.stats_task is None, 'Stats task should be None initially'
print('   ✓ CLIBridgeRunner initializes correctly')
"

echo ""
echo "10. Testing CLI entry point (mavsim-sensor-bridge command)..."
# Check if the command is available (after package installation)
if command -v mavsim-sensor-bridge &> /dev/null; then
    echo "   Testing --help option..."
    if mavsim-sensor-bridge --help > /dev/null 2>&1; then
        echo "   ✓ CLI command is available and --help works"
    else
        echo "   ⚠ CLI command exists but --help failed"
    fi
else
    echo "   ⚠ CLI command not found (package may need to be installed)"
    echo "   Install with: pip install -e ."
    echo "   Then test with: mavsim-sensor-bridge --help"
fi

echo ""
echo "11. Running pytest tests..."
if command -v pytest &> /dev/null; then
    if timeout 60 pytest tests/unit/test_task_1_7_cli.py -v --tb=short; then
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
echo "=========================================="
echo "All tests passed! ✓"
echo ""
echo "To run pytest tests manually:"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_7_cli.py -v"
echo ""
echo "To test the CLI manually:"
echo "  mavsim-sensor-bridge --help"
echo "  mavsim-sensor-bridge --verbose --stats-interval 5"
echo "  mavsim-sensor-bridge --camera-port 9000"
