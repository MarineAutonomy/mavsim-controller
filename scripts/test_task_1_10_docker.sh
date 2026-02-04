#!/bin/bash
# Docker test script for Task 1.10: Phase 1 Integration Test
# Starts the bridge in Docker, runs the Phase 1 integration test, then cleans up.

set -e

echo "============================================="
echo "Testing Task 1.10: Phase 1 Integration Test"
echo "============================================="
echo ""

# Run from sensor_bridge root (script may be in scripts/)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

if ! docker compose version &> /dev/null && ! docker-compose version &> /dev/null; then
    echo "Error: docker-compose is not installed or not in PATH"
    exit 1
fi

if docker compose version &> /dev/null; then
    DOCKER_COMPOSE="docker compose"
else
    DOCKER_COMPOSE="docker-compose"
fi

echo "1. Building Docker images (if needed)..."
$DOCKER_COMPOSE -f docker-compose.test.yml build --quiet 2>/dev/null || \
$DOCKER_COMPOSE -f docker-compose.test.yml build || {
    echo "Error: Docker build failed"
    exit 1
}

echo ""
echo "2. Running Phase 1 integration tests..."
echo "   (Bridge will start and wait for healthy, then tests run)"
echo ""
EXIT_CODE=0
$DOCKER_COMPOSE -f docker-compose.test.yml run --rm test pytest tests/integration/test_phase_1_integration.py -v || EXIT_CODE=$?

echo ""
echo "3. Stopping and removing containers..."
$DOCKER_COMPOSE -f docker-compose.test.yml down

echo ""
if [ "$EXIT_CODE" -eq 0 ]; then
    echo "============================================="
    echo "Phase 1 integration test completed successfully! ✓"
    echo "============================================="
else
    echo "============================================="
    echo "Phase 1 integration test failed (exit code $EXIT_CODE)"
    echo "============================================="
fi
echo ""
echo "To run manually:"
echo "  docker-compose -f docker-compose.test.yml run test pytest tests/integration/test_phase_1_integration.py -v"
echo ""
exit $EXIT_CODE
