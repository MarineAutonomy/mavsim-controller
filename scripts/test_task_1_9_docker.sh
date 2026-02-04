#!/bin/bash
# Docker test script for Task 1.9: Docker Development Environment
# This script uses docker-compose to test the bridge in a containerized environment

set -e

echo "=========================================="
echo "Testing Task 1.9: Docker Development Environment"
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

echo "1. Building Docker images..."
$DOCKER_COMPOSE -f docker-compose.test.yml build || {
    echo "Error: Docker build failed"
    exit 1
}

echo ""
echo "2. Starting bridge service and running tests..."
echo "   (This will start the bridge container and run integration tests)"
echo ""

# Run docker-compose up which will:
# - Start the bridge service
# - Wait for it to be healthy
# - Run the test service
# - Exit when test completes
$DOCKER_COMPOSE -f docker-compose.test.yml up --abort-on-container-exit --exit-code-from test || {
    echo ""
    echo "Error: Docker test failed"
    echo ""
    echo "Cleaning up containers..."
    $DOCKER_COMPOSE -f docker-compose.test.yml down
    exit 1
}

echo ""
echo "3. Cleaning up containers..."
$DOCKER_COMPOSE -f docker-compose.test.yml down

echo ""
echo "=========================================="
echo "Docker test completed successfully! ✓"
echo "=========================================="
echo ""
echo "To run manually:"
echo "  cd sensor_bridge && docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit"
echo ""
echo "To run only the bridge (for manual testing):"
echo "  docker-compose -f docker-compose.test.yml up bridge"
echo ""
