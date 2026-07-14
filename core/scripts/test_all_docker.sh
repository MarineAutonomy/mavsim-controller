#!/bin/bash
# Unified Docker test script for all client controller tests
# This script builds one integrated container and runs all dockerized tests
# Run from repository root

set -e

echo "============================================="
echo "Testing All Client Controller Functionality"
echo "============================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

# Standard image name for integrated container
IMAGE_NAME="mavlab/mavsim-controller:latest"

echo "1. Building integrated Docker image..."
echo "   Image: $IMAGE_NAME"
echo "   This image supports both running controller and running tests"
echo ""

cd "$ROOT_DIR"
docker build -f "$ROOT_DIR/user_repo_new/Dockerfile" -t "$IMAGE_NAME" "$ROOT_DIR" || {
    echo "Error: Failed to build Docker image"
    exit 1
}

echo ""
echo "2. Running all Phase 2 unit tests..."
echo ""

EXIT_CODE=0

# Task 2.1: Recording Command Polling
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Task 2.1: Recording Command Polling"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_1_polling.py -v" || EXIT_CODE=$?

echo ""

# Task 2.2: Auto-Start Recording When Commanded
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Task 2.2: Auto-Start Recording When Commanded"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_2_auto_start.py -v" || EXIT_CODE=$?

echo ""

# Task 2.3: ROS2 Bag Recording Service
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Task 2.3: ROS2 Bag Recording Service"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/recording_service.py:/app/recording_service.py:ro" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_3_recording_service.py -v" || EXIT_CODE=$?

echo ""

# Task 2.4: Recording Status Reporting
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Task 2.4: Recording Status Reporting"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_4_status_reporting.py -v" || EXIT_CODE=$?

echo ""

# Task 2.5: Publish Sensor Data to Local ROS2 (Client Controller Only)
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Task 2.5: Local ROS2 Camera Publish"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
docker run --rm --entrypoint="" \
    -e PYTHONPATH=/app/sensor_bridge_setup \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/tests/unit/test_task_2_5_local_ros2_publish.py:/app/tests/unit/test_task_2_5_local_ros2_publish.py:ro" \
    -v "$ROOT_DIR/sensor_bridge/mavsim_sensor_bridge:/app/sensor_bridge_setup/mavsim_sensor_bridge:ro" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_5_local_ros2_publish.py -v" || EXIT_CODE=$?

echo ""

# Integration Test (E2E)
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Integration Test: End-to-End Client Recording"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if backend container is running for E2E test
BACKEND_CONTAINER="mavsim-backend"
if docker ps --format '{{.Names}}' | grep -q "^${BACKEND_CONTAINER}$"; then
    # Get backend container network
    BACKEND_NETWORK=$(docker inspect "$BACKEND_CONTAINER" --format '{{range $net, $conf := .NetworkSettings.Networks}}{{$net}}{{end}}' | head -1)
    if [ -n "$BACKEND_NETWORK" ]; then
        echo "   Backend found, running E2E test with network access..."
        docker run --rm --entrypoint="" \
            --network "$BACKEND_NETWORK" \
            -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
            -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
            -v "$ROOT_DIR/user_repo_new/core/recording_service.py:/app/recording_service.py:ro" \
            -e BACKEND_URL="http://${BACKEND_CONTAINER}:5000" \
            "$IMAGE_NAME" \
            bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/integration/test_client_recording_e2e.py -v" || EXIT_CODE=$?
    else
        echo "   Backend network not found, skipping E2E test (requires backend running)"
    fi
else
    echo "   Backend not running, skipping E2E test (requires backend running)"
    echo "   To run E2E test: docker compose up -d backend"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ $EXIT_CODE -eq 0 ]; then
    echo "All tests passed! ✓"
    echo ""
    echo "Container '$IMAGE_NAME' is ready for:"
    echo "  - Running client controller: docker run --rm -v ./my_controller.py:/app/my_controller.py $IMAGE_NAME --code ABC123"
    echo "  - Running tests: docker run --rm --entrypoint=\"\" $IMAGE_NAME bash -c \"...\""
else
    echo "Some tests failed! ✗"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

exit $EXIT_CODE















