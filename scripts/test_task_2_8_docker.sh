#!/bin/bash
# Docker test script for Task 2.8: Phase 2 Integration Test
# This script tests the end-to-end camera streaming flow:
# - Controller container starts with bridge
# - Mock browser connects to bridge ports
# - Camera frames flow: mock browser → bridge → controller callback
# - Multiple cameras work simultaneously
# - Bridge survives reconnection
#
# Uses the main docker-compose.yml services (does NOT create separate compose files)
# Reuses existing containers if running, otherwise starts them from main docker-compose.yml
# Does NOT stop containers that are part of the main docker-compose setup

set -e

echo "============================================="
echo "Testing Task 2.8: Phase 2 Integration Test"
echo "============================================="
echo ""

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT_DIR="$(cd "$SENSOR_BRIDGE_DIR/.." && pwd)"
cd "$ROOT_DIR"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

# Check for docker-compose
if docker compose version &> /dev/null 2>&1; then
    DOCKER_COMPOSE="docker compose"
elif command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
else
    echo "Error: docker-compose is not installed or not in PATH"
    exit 1
fi

# Check if sensor-bridge container is already running
SENSOR_BRIDGE_RUNNING=false
if docker ps --format '{{.Names}}' | grep -q "^mavsim-sensor-bridge$"; then
    SENSOR_BRIDGE_RUNNING=true
    echo "✓ Using existing sensor-bridge container (mavsim-sensor-bridge)"
else
    echo "Sensor-bridge container not running, starting from main docker-compose.yml..."
    echo ""
    
    # Start sensor-bridge service from main docker-compose.yml
    echo "1. Starting sensor-bridge service..."
    $DOCKER_COMPOSE up -d sensor-bridge || {
        echo "Error: Failed to start sensor-bridge service"
        exit 1
    }
    
    # Wait for container to be healthy
    echo "   Waiting for sensor-bridge to be healthy..."
    MAX_WAIT=30
    WAIT_COUNT=0
    while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
        if docker ps --format '{{.Names}} {{.Status}}' | grep -q "mavsim-sensor-bridge.*healthy"; then
            echo "   ✓ Sensor-bridge is healthy"
            SENSOR_BRIDGE_RUNNING=true
            break
        fi
        sleep 1
        WAIT_COUNT=$((WAIT_COUNT + 1))
    done
    
    if [ "$SENSOR_BRIDGE_RUNNING" = false ]; then
        echo "   ⚠ Sensor-bridge container started but may not be fully ready"
        echo "   Continuing with test (will wait longer in test)..."
        SENSOR_BRIDGE_RUNNING=true
    fi
fi

echo ""
echo "2. Running Phase 2 integration tests..."
echo "   Test file: sensor_bridge/tests/integration/test_phase_2_camera_e2e.py"
echo ""

# Determine network name from docker-compose
NETWORK_NAME="mavsim_mavsim-network"
if ! docker network ls --format '{{.Name}}' | grep -q "^${NETWORK_NAME}$"; then
    # Try alternative network name format
    NETWORK_NAME="mavsim-network"
fi

# Build test image if needed (using sensor_bridge Dockerfile.dev)
TEST_IMAGE="mavsim-sensor-bridge-test:task-2.8"
echo "   Building test image..."
docker build -f "$SENSOR_BRIDGE_DIR/Dockerfile.dev" -t "$TEST_IMAGE" "$SENSOR_BRIDGE_DIR" || {
    echo "Error: Failed to build test image"
    exit 1
}

# Run integration tests in a container connected to the same network
# Note: The test starts its own bridge instance (simulating a controller),
# so BRIDGE_HOST should be localhost (the test container's own bridge)
echo "   Running tests..."
EXIT_CODE=0
docker run --rm \
    --network "$NETWORK_NAME" \
    -e BRIDGE_HOST=localhost \
    -e BRIDGE_PORT=8765 \
    -e BRIDGE_READY_DELAY=2.0 \
    -v "$ROOT_DIR/sensor_bridge:/app" \
    -w /app \
    "$TEST_IMAGE" \
    pytest tests/integration/test_phase_2_camera_e2e.py -v || EXIT_CODE=$?

echo ""
if [ "$EXIT_CODE" -eq 0 ]; then
    echo "============================================="
    echo "Phase 2 integration test completed successfully! ✓"
    echo "============================================="
    echo ""
    echo "Summary:"
    echo "  ✓ Camera frames flow: mock browser → bridge → controller callback"
    echo "  ✓ Multiple cameras work simultaneously"
    echo "  ✓ Bridge survives reconnection"
    echo ""
else
    echo "============================================="
    echo "Phase 2 integration test failed (exit code $EXIT_CODE)"
    echo "============================================="
    echo ""
    echo "Troubleshooting:"
    echo "  - Check that sensor-bridge container is running: docker ps | grep sensor-bridge"
    echo "  - Check sensor-bridge logs: docker logs mavsim-sensor-bridge"
    echo "  - Verify network connectivity: docker network inspect $NETWORK_NAME"
    echo ""
fi

# Note: We do NOT stop the sensor-bridge container as it's part of main docker-compose.yml
# The user may want to keep it running for other services

echo "Note: sensor-bridge container remains running (part of main docker-compose.yml)"
echo ""
echo "To run tests manually:"
echo "  docker run --rm --network $NETWORK_NAME \\"
echo "    -e BRIDGE_HOST=localhost -e BRIDGE_PORT=8765 \\"
echo "    -v $ROOT_DIR/sensor_bridge:/app -w /app \\"
echo "    $TEST_IMAGE pytest tests/integration/test_phase_2_camera_e2e.py -v"
echo ""

exit $EXIT_CODE

