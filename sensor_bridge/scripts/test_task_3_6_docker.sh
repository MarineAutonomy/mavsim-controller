#!/bin/bash
# Docker test script for Task 3.6: Phase 3 Lidar Integration Test
# This script tests the end-to-end lidar point cloud streaming flow:
# - Bridge starts with lidar server enabled
# - Mock browser client connects to bridge lidar port
# - Point clouds flow: mock client → bridge → controller callback
# - Various point counts work (1K, 10K, 100K)
# - High throughput: 100K points at 10Hz
# - Multiple vessels work simultaneously
# - Bridge survives reconnection
#
# Uses the main docker-compose.yml services (does NOT create separate compose files)
# Reuses existing containers if running, otherwise starts them from main docker-compose.yml
# Does NOT stop containers that are part of the main docker-compose setup

set -e

echo "============================================="
echo "Testing Task 3.6: Phase 3 Lidar Integration Test"
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

# Determine network name from docker-compose
NETWORK_NAME="mavsim_mavsim-network"
if ! docker network ls --format '{{.Name}}' | grep -q "^${NETWORK_NAME}$"; then
    # Try alternative network name format
    NETWORK_NAME="mavsim-network"
    if ! docker network ls --format '{{.Name}}' | grep -q "^${NETWORK_NAME}$"; then
        echo "Docker network not found, creating it..."
        docker network create "$NETWORK_NAME" 2>/dev/null || true
        NETWORK_NAME="mavsim-network"
    fi
fi

echo "Using Docker network: $NETWORK_NAME"
echo ""

# Build test image using sensor_bridge Dockerfile.dev
TEST_IMAGE="mavsim-sensor-bridge-test:task-3.6"
echo "1. Building test image..."
docker build -f "$SENSOR_BRIDGE_DIR/Dockerfile.dev" -t "$TEST_IMAGE" "$SENSOR_BRIDGE_DIR" || {
    echo "Error: Failed to build test image"
    exit 1
}

echo ""
echo "2. Running Phase 3 lidar integration tests..."
echo "   Test file: sensor_bridge/tests/integration/test_phase_3_lidar_e2e.py"
echo ""

# Run integration tests in a container connected to the Docker network.
# The test starts its own bridge instance (simulating a controller),
# so BRIDGE_HOST should be localhost (the test container's own bridge).
EXIT_CODE=0
docker run --rm \
    --network "$NETWORK_NAME" \
    -e BRIDGE_HOST=localhost \
    -e LIDAR_PORT=8766 \
    -e BRIDGE_READY_DELAY=2.0 \
    -e PYTHONUNBUFFERED=1 \
    -v "$ROOT_DIR/sensor_bridge:/app" \
    -w /app \
    "$TEST_IMAGE" \
    pytest tests/integration/test_phase_3_lidar_e2e.py -v || EXIT_CODE=$?

echo ""
if [ "$EXIT_CODE" -eq 0 ]; then
    echo "============================================="
    echo "Phase 3 lidar integration test completed successfully! ✓"
    echo "============================================="
    echo ""
    echo "Summary:"
    echo "  ✓ Lidar server accepts WebSocket connections"
    echo "  ✓ Point clouds flow: mock client → bridge → controller callback"
    echo "  ✓ Various point counts (1K, 10K, 100K)"
    echo "  ✓ High throughput: 100K points at 10Hz"
    echo "  ✓ Multiple vessels work simultaneously"
    echo "  ✓ Bridge survives reconnection"
    echo "  ✓ Bridge stats updated correctly"
    echo "  ✓ Lidar and camera can coexist"
    echo ""
else
    echo "============================================="
    echo "Phase 3 lidar integration test failed (exit code $EXIT_CODE)"
    echo "============================================="
    echo ""
    echo "Troubleshooting:"
    echo "  - Check Docker image build: docker build -f sensor_bridge/Dockerfile.dev -t $TEST_IMAGE sensor_bridge/"
    echo "  - Run tests manually inside container:"
    echo "    docker run --rm -it --network $NETWORK_NAME \\"
    echo "      -v $ROOT_DIR/sensor_bridge:/app -w /app \\"
    echo "      $TEST_IMAGE bash"
    echo "  - Then inside: pytest tests/integration/test_phase_3_lidar_e2e.py -v -s"
    echo ""
fi

# Note: We do NOT stop any containers as they may be part of main docker-compose.yml
echo "Note: No main docker-compose containers were stopped."
echo ""
echo "To run tests manually:"
echo "  docker run --rm --network $NETWORK_NAME \\"
echo "    -e BRIDGE_HOST=localhost -e LIDAR_PORT=8766 \\"
echo "    -v $ROOT_DIR/sensor_bridge:/app -w /app \\"
echo "    $TEST_IMAGE pytest tests/integration/test_phase_3_lidar_e2e.py -v"
echo ""

exit $EXIT_CODE

