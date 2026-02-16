#!/bin/bash
# Docker test script for Task 3.1: Lidar Sensor Server
# This script builds and runs the Docker container to test task 3.1

set -e

echo "============================================="
echo "Testing Task 3.1 in Docker Container"
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

echo "1. Building Docker image..."
docker build -f Dockerfile.dev -t mavsim-sensor-bridge-test:latest . || {
    echo "Error: Docker build failed"
    exit 1
}

echo ""
echo "2. Running basic tests in Docker container..."
echo ""

# Run the basic test script in the container
docker run --rm \
    -v "$(pwd):/app" \
    -w /app \
    -e PYTHONUNBUFFERED=1 \
    mavsim-sensor-bridge-test:latest \
    bash scripts/test_task_3_1.sh

echo ""
echo "3. Running pytest unit tests in Docker container..."
echo ""

# Run pytest tests in the container
docker run --rm \
    -v "$(pwd):/app" \
    -w /app \
    -e PYTHONUNBUFFERED=1 \
    mavsim-sensor-bridge-test:latest \
    pytest tests/unit/test_task_3_1_lidar_server.py -v

echo ""
echo "============================================="
echo "Docker test completed!"
echo "============================================="







