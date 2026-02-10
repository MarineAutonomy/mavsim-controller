#!/bin/bash
# Docker test script for Task 3.2: Update SensorBridge for Lidar
# Builds and runs the sensor_bridge dev image to verify task 3.2 in a container.
# Uses existing Dockerfile.dev; does not create separate docker-compose files.

set -e

echo "============================================="
echo "Testing Task 3.2 (SensorBridge Lidar) in Docker"
echo "============================================="
echo ""

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

echo "1. Building Docker image (if needed)..."
docker build -f Dockerfile.dev -t mavsim-sensor-bridge-test:latest . || {
    echo "Error: Docker build failed"
    exit 1
}

echo ""
echo "2. Running basic script tests in Docker container..."
docker run --rm \
    -v "$(pwd):/app" \
    -w /app \
    -e PYTHONUNBUFFERED=1 \
    mavsim-sensor-bridge-test:latest \
    bash scripts/test_task_3_2.sh

echo ""
echo "3. Running pytest unit tests in Docker container..."
docker run --rm \
    -v "$(pwd):/app" \
    -w /app \
    -e PYTHONUNBUFFERED=1 \
    mavsim-sensor-bridge-test:latest \
    pytest tests/unit/test_task_3_2_bridge_lidar.py -v

echo ""
echo "============================================="
echo "Task 3.2 Docker test completed successfully!"
echo "============================================="
