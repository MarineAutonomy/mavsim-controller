#!/bin/bash
# Docker test script for Phase 2: Client Infrastructure
# This script runs all Phase 2 tests inside the controller Docker container
# Run from repository root

set -e

echo "============================================="
echo "Testing Phase 2: Client Infrastructure"
echo "============================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

echo "Building integrated Docker image..."
docker build -f "$ROOT_DIR/user_repo_new/Dockerfile" -t mavsim-controller:latest "$ROOT_DIR" || {
    echo "Error: Failed to build Docker image"
    exit 1
}

echo ""
echo "Running all Phase 2 tests..."
echo ""

EXIT_CODE=0

echo "1. Task 2.1: Recording Command Polling"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    mavsim-controller:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_1_polling.py -v" || EXIT_CODE=$?

echo ""
echo "2. Task 2.2: Auto-Start Recording When Commanded"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    mavsim-controller:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_2_auto_start.py -v" || EXIT_CODE=$?

echo ""
echo "3. Task 2.3: ROS2 Bag Recording Service"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/recording_service.py:/app/recording_service.py:ro" \
    mavsim-controller:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_3_recording_service.py -v" || EXIT_CODE=$?

echo ""
echo "4. Task 2.4: Recording Status Reporting"
docker run --rm --entrypoint="" \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    mavsim-controller:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_4_status_reporting.py -v" || EXIT_CODE=$?

echo ""
echo "5. Task 2.5: Publish Sensor Data to Local ROS2 (Client Controller Only)"
docker run --rm --entrypoint="" \
    -e PYTHONPATH=/app/sensor_bridge_setup \
    -v "$ROOT_DIR/user_repo_new/core/base_controller.py:/app/base_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/python_controller.py:/app/python_controller.py:ro" \
    -v "$ROOT_DIR/user_repo_new/core/tests/unit/test_task_2_5_local_ros2_publish.py:/app/tests/unit/test_task_2_5_local_ros2_publish.py:ro" \
    -v "$ROOT_DIR/sensor_bridge/mavsim_sensor_bridge:/app/sensor_bridge_setup/mavsim_sensor_bridge:ro" \
    mavsim-controller:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m pytest /app/tests/unit/test_task_2_5_local_ros2_publish.py -v" || EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "============================================="
    echo "Phase 2 all tests passed! ✓"
    echo "============================================="
else
    echo "============================================="
    echo "Phase 2 some tests failed! ✗"
    echo "============================================="
fi

exit $EXIT_CODE

