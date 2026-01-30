#!/bin/bash
# Test script for Task 1.10: Phase 1 Integration Test
# Validates that Phase 1 integration test file and Docker setup exist and are well-formed.

set -e

echo "Testing Task 1.10: Phase 1 Integration Test"
echo "============================================="

cd "$(dirname "$0")"

echo ""
echo "1. Checking docker-compose.test.yml exists..."
if [ ! -f "docker-compose.test.yml" ]; then
    echo "   ✗ Error: docker-compose.test.yml not found"
    exit 1
fi
echo "   ✓ docker-compose.test.yml exists"

echo ""
echo "2. Checking Dockerfile.dev exists..."
if [ ! -f "Dockerfile.dev" ]; then
    echo "   ✗ Error: Dockerfile.dev not found"
    exit 1
fi
echo "   ✓ Dockerfile.dev exists"

echo ""
echo "3. Checking Phase 1 integration test file exists..."
if [ ! -f "tests/integration/test_phase_1_integration.py" ]; then
    echo "   ✗ Error: tests/integration/test_phase_1_integration.py not found"
    exit 1
fi
echo "   ✓ test_phase_1_integration.py exists"

echo ""
echo "4. Validating Phase 1 integration test structure..."
python3 -c "
import sys

with open('tests/integration/test_phase_1_integration.py', 'r') as f:
    content = f.read()

required_tests = [
    'test_camera_server_end_to_end',
    'test_multiple_cameras',
    'test_high_throughput',
    'test_graceful_connection_close',
]

for test_name in required_tests:
    if f'def {test_name}' in content:
        print(f'   ✓ Test function {test_name} found')
    else:
        print(f'   ✗ Test function {test_name} not found')
        sys.exit(1)

required_imports = ['pytest', 'websockets', 'pack_camera_frame']
for imp in required_imports:
    if imp in content:
        print(f'   ✓ Import {imp} found')
    else:
        print(f'   ✗ Import {imp} not found')
        sys.exit(1)
"

echo ""
echo "5. Validating Docker Compose configuration..."
if command -v docker &> /dev/null; then
    if docker compose version &> /dev/null 2>&1; then
        if docker compose -f docker-compose.test.yml config > /dev/null 2>&1; then
            echo "   ✓ docker-compose.test.yml is valid"
        else
            echo "   ✗ docker-compose.test.yml validation failed"
            exit 1
        fi
    elif docker-compose version &> /dev/null 2>&1; then
        if docker-compose -f docker-compose.test.yml config > /dev/null 2>&1; then
            echo "   ✓ docker-compose.test.yml is valid"
        else
            echo "   ✗ docker-compose.test.yml validation failed"
            exit 1
        fi
    else
        echo "   ⚠ docker compose not available, skipping config validation"
    fi
else
    echo "   ⚠ Docker not available, skipping config validation"
fi

echo ""
echo "============================================="
echo "All validation tests passed! ✓"
echo ""
echo "To run Phase 1 integration test in Docker:"
echo "  cd sensor_bridge"
echo "  bash test_task_1_10_docker.sh"
echo ""
echo "Or manually:"
echo "  docker-compose -f docker-compose.test.yml run test pytest tests/integration/test_phase_1_integration.py -v"
echo ""
