#!/bin/bash
# Docker test script for Task 4.1: End-to-End Client Recording Integration Test
# This script runs the E2E integration test in a Docker container
# Requires: Backend running (from docker-compose), client container built
# Run from repository root

set -e

echo "============================================="
echo "Testing Task 4.1: End-to-End Client Recording"
echo "============================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: docker-compose is not installed or not in PATH"
    exit 1
fi

# Use 'docker compose' if available, otherwise 'docker-compose'
if docker compose version &> /dev/null; then
    DOCKER_COMPOSE="docker compose"
else
    DOCKER_COMPOSE="docker-compose"
fi

# Check if backend container is running
echo "1. Checking backend services..."
BACKEND_CONTAINER="mavsim-backend"
if ! docker ps --format '{{.Names}}' | grep -q "^${BACKEND_CONTAINER}$"; then
    echo "   Backend container not found. Starting services from docker-compose.yml..."
    cd "$ROOT_DIR"
    $DOCKER_COMPOSE up -d backend || {
        echo "Error: Failed to start backend container"
        exit 1
    }
    echo "   Waiting for backend to be healthy..."
    sleep 5
else
    echo "   Backend container is already running."
fi

# Get backend container network
BACKEND_NETWORK=$(docker inspect "$BACKEND_CONTAINER" --format '{{range $net, $conf := .NetworkSettings.Networks}}{{$net}}{{end}}' | head -1)
if [ -z "$BACKEND_NETWORK" ]; then
    echo "Error: Could not determine backend network"
    exit 1
fi

echo "   Backend network: $BACKEND_NETWORK"
echo "   Backend URL: http://${BACKEND_CONTAINER}:5000"

echo ""
echo "2. Building integrated Docker image..."
docker build -f "$ROOT_DIR/user_repo_new/Dockerfile" -t mavsim-controller:latest "$ROOT_DIR" || {
    echo "Error: Failed to build Docker image"
    exit 1
}

echo ""
echo "3. Running E2E integration test in client container..."
echo "   Test file: tests/integration/test_client_recording_e2e.py"
echo ""

# Run pytest inside the container (override entrypoint)
EXIT_CODE=0
docker run --rm --entrypoint="" \
    --network "$BACKEND_NETWORK" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/recording_service.py:/app/recording_service.py:ro" \
    -e BACKEND_URL="http://${BACKEND_CONTAINER}:5000" \
    mavsim-controller:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/integration/test_client_recording_e2e.py -v" || EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "============================================="
    echo "Task 4.1 E2E test passed! ✓"
    echo "============================================="
else
    echo "============================================="
    echo "Task 4.1 E2E test failed! ✗"
    echo "============================================="
fi

exit $EXIT_CODE

