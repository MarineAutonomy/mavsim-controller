#!/bin/bash
# Docker test script for Task 1.8: Bridge Configuration
# This script builds and runs the Docker container to test task 1.8

set -e

echo "=========================================="
echo "Testing Task 1.8 in Docker Container"
echo "=========================================="
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
echo "2. Running tests in Docker container..."
echo ""

# Run the test script in the container
docker run --rm \
    -v "$(pwd):/app" \
    -w /app \
    -e PYTHONUNBUFFERED=1 \
    mavsim-sensor-bridge-test:latest \
    bash scripts/test_task_1_8.sh

echo ""
echo "=========================================="
echo "Docker test completed!"
echo "=========================================="
