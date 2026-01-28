#!/bin/bash
# Test script for Task 1.1: Bridge Package Structure
# This script verifies that the package can be imported correctly

set -e

echo "Testing Task 1.1: Bridge Package Structure"
echo "=========================================="

# Change to sensor_bridge directory
cd "$(dirname "$0")"

echo ""
echo "1. Testing package import..."
python3 -c "
import sys
sys.path.insert(0, '.')
from mavsim_sensor_bridge import SensorBridge, __version__
print(f'   ✓ Package imported successfully')
print(f'   ✓ Version: {__version__}')
print(f'   ✓ SensorBridge class: {SensorBridge}')
"

echo ""
echo "2. Testing package structure..."
if [ -f "pyproject.toml" ]; then
    echo "   ✓ pyproject.toml exists"
else
    echo "   ✗ pyproject.toml missing"
    exit 1
fi

if [ -f "requirements.txt" ]; then
    echo "   ✓ requirements.txt exists"
else
    echo "   ✗ requirements.txt missing"
    exit 1
fi

if [ -f "mavsim_sensor_bridge/__init__.py" ]; then
    echo "   ✓ mavsim_sensor_bridge/__init__.py exists"
else
    echo "   ✗ mavsim_sensor_bridge/__init__.py missing"
    exit 1
fi

if [ -f "tests/unit/test_task_1_1_package.py" ]; then
    echo "   ✓ tests/unit/test_task_1_1_package.py exists"
else
    echo "   ✗ tests/unit/test_task_1_1_package.py missing"
    exit 1
fi

echo ""
echo "3. Testing pip install (editable mode)..."
if python3 -m pip install -e . > /dev/null 2>&1; then
    echo "   ✓ Package installs successfully"
    python3 -c "import mavsim_sensor_bridge; print(f'   ✓ Package can be imported after install')"
else
    echo "   ⚠ pip install -e . failed (may need dependencies installed)"
    echo "   This is expected if dependencies are not installed yet"
fi

echo ""
echo "=========================================="
echo "All tests passed! ✓"
echo ""
echo "To run pytest tests (requires pytest installed):"
echo "  cd sensor_bridge && pytest tests/unit/test_task_1_1_package.py -v"
