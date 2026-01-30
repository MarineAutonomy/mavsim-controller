#!/bin/bash
# Test script for Task 1.9: Docker Development Environment
# This script verifies that Docker setup works correctly

set -e

echo "Testing Task 1.9: Docker Development Environment"
echo "================================================="

# Change to sensor_bridge directory
cd "$(dirname "$0")"

echo ""
echo "1. Checking Dockerfile.dev exists..."
if [ ! -f "Dockerfile.dev" ]; then
    echo "   ✗ Error: Dockerfile.dev not found"
    exit 1
fi
echo "   ✓ Dockerfile.dev exists"

echo ""
echo "2. Checking docker-compose.test.yml exists..."
if [ ! -f "docker-compose.test.yml" ]; then
    echo "   ✗ Error: docker-compose.test.yml not found"
    exit 1
fi
echo "   ✓ docker-compose.test.yml exists"

echo ""
echo "3. Validating Dockerfile.dev structure..."
python3 -c "
import re

with open('Dockerfile.dev', 'r') as f:
    content = f.read()

# Check for required elements
checks = [
    ('FROM python', 'Python base image'),
    ('WORKDIR /app', 'Working directory'),
    ('COPY requirements.txt', 'Requirements copy'),
    ('RUN pip install', 'Pip install'),
    ('ENV PYTHONUNBUFFERED', 'Python unbuffered env'),
]
for pattern, desc in checks:
    if pattern in content:
        print(f'   ✓ {desc}')
    else:
        print(f'   ✗ Missing: {desc}')
        exit(1)

# Check EXPOSE includes all WebSocket ports (single or multiple EXPOSE lines)
if 'EXPOSE' not in content:
    print('   ✗ Missing: EXPOSE directive')
    exit(1)
for port in [8765, 8766, 8767, 8768, 8769]:
    if str(port) not in content:
        print(f'   ✗ Missing: Port {port} in EXPOSE')
        exit(1)
print('   ✓ Ports 8765-8769 exposed')
"

echo ""
echo "4. Validating docker-compose.test.yml structure..."
python3 -c "
import yaml
import sys

try:
    with open('docker-compose.test.yml', 'r') as f:
        compose = yaml.safe_load(f)
    
    # Check for required services
    if 'services' not in compose:
        print('   ✗ Error: No services section found')
        sys.exit(1)
    
    services = compose['services']
    
    # Check bridge service
    if 'bridge' not in services:
        print('   ✗ Error: bridge service not found')
        sys.exit(1)
    
    bridge = services['bridge']
    checks = [
        ('build', 'Build configuration'),
        ('ports', 'Port mappings'),
        ('volumes', 'Volume mounts'),
        ('command', 'Command override'),
        ('healthcheck', 'Health check'),
    ]
    
    for key, desc in checks:
        if key in bridge:
            print(f'   ✓ Bridge service has {desc}')
        else:
            print(f'   ✗ Bridge service missing {desc}')
            sys.exit(1)
    
    # Check ports
    if '8765:8765' not in str(bridge.get('ports', [])):
        print('   ✗ Error: Port 8765 not mapped')
        sys.exit(1)
    print('   ✓ Port 8765 mapped correctly')
    
    # Check test service
    if 'test' not in services:
        print('   ✗ Error: test service not found')
        sys.exit(1)
    
    test = services['test']
    if 'depends_on' not in test:
        print('   ✗ Error: test service missing depends_on')
        sys.exit(1)
    
    if 'bridge' not in test.get('depends_on', {}):
        print('   ✗ Error: test service does not depend on bridge')
        sys.exit(1)
    
    print('   ✓ Test service depends on bridge')
    
    if 'BRIDGE_HOST' not in str(test.get('environment', [])):
        print('   ✗ Error: BRIDGE_HOST environment variable not set')
        sys.exit(1)
    print('   ✓ BRIDGE_HOST environment variable set')
    
except ImportError:
    print('   ⚠ PyYAML not installed, skipping YAML validation')
    print('   Install with: pip install pyyaml')
except Exception as e:
    print(f'   ✗ Error validating docker-compose.test.yml: {e}')
    sys.exit(1)
"

echo ""
echo "5. Checking integration test file exists..."
if [ ! -f "tests/integration/test_task_1_9_docker.py" ]; then
    echo "   ✗ Error: tests/integration/test_task_1_9_docker.py not found"
    exit 1
fi
echo "   ✓ Integration test file exists"

echo ""
echo "6. Validating integration test structure..."
python3 -c "
import sys
import ast

with open('tests/integration/test_task_1_9_docker.py', 'r') as f:
    content = f.read()

# Check for required test functions
required_tests = [
    'test_bridge_accepts_external_connection',
    'test_bridge_multiple_connections',
    'test_bridge_port_exposed',
    'test_bridge_health_check',
]

for test_name in required_tests:
    if f'def {test_name}' in content:
        print(f'   ✓ Test function {test_name} found')
    else:
        print(f'   ✗ Test function {test_name} not found')
        sys.exit(1)

# Check for required imports
required_imports = [
    'pytest',
    'websockets',
    'pack_camera_frame',
]

for imp in required_imports:
    if imp in content:
        print(f'   ✓ Import {imp} found')
    else:
        print(f'   ⚠ Import {imp} not found (may be acceptable)')
"

echo ""
echo "7. Validating Docker Compose configuration..."
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
echo "================================================="
echo "All validation tests passed! ✓"
echo ""
echo "To run the full Docker test:"
echo "  cd sensor_bridge"
echo "  bash test_task_1_9_docker.sh"
echo ""
echo "Or manually with docker-compose:"
echo "  docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit"
echo ""
