#!/bin/bash
# Docker test script for Task 2.5: Publish Sensor Data to Local ROS2 (Client Controller Only)
# This script runs pytest tests inside the controller Docker container with ROS2 Humble.
# Run from repository root.

set -e

echo "============================================="
echo "Testing Task 2.5: Local ROS2 Camera Publish"
echo "============================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

# Check if unified image exists, build if not
IMAGE_NAME="mavlab/mavsim-controller:latest"
if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "1. Building integrated Docker image..."
    docker build -f "$ROOT_DIR/user_repo_new/Dockerfile" -t "$IMAGE_NAME" "$ROOT_DIR" || {
        echo "Error: Failed to build Docker image"
        exit 1
    }
else
    echo "1. Using existing Docker image: $IMAGE_NAME"
fi

echo ""
echo "2. Running pytest tests in Docker container..."
echo "   Test file: /app/tests/unit/test_task_2_5_local_ros2_publish.py"
echo "   Note: Tests require ROS2 Humble environment"
echo ""

# Run pytest inside the container; mount sensor_bridge so new ros2_publisher is used.
# PYTHONPATH ensures mavsim_sensor_bridge is loaded from mounted sensor_bridge (container has websockets from requirements.txt).
EXIT_CODE=0
docker run --rm --entrypoint="" \
    -e PYTHONPATH=/app/sensor_bridge_setup \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/tests/unit/test_task_2_5_local_ros2_publish.py:/app/tests/unit/test_task_2_5_local_ros2_publish.py:ro" \
    -v "$ROOT_DIR/sensor_bridge/mavsim_sensor_bridge:/app/sensor_bridge_setup/mavsim_sensor_bridge:ro" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_5_local_ros2_publish.py -v" || EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "============================================="
    echo "Task 2.5 tests passed! ✓"
    echo "============================================="
else
    echo "============================================="
    echo "Task 2.5 tests failed! ✗"
    echo "============================================="
fi

exit $EXIT_CODE
