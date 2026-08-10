#!/bin/bash
# Docker test script for Task 2.1: SensorStreamManager protocol (bridge side)
# Runs Task 2.1 integration tests inside the existing bridge container (no separate test container).
# Run from anywhere; script dir is scripts/, sensor_bridge root is parent.

set -e

echo "============================================="
echo "Testing Task 2.1 (Bridge) in Existing Container"
echo "============================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SENSOR_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SENSOR_BRIDGE_DIR"

if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null 2>&1; then
    echo "Error: docker-compose is not installed or not in PATH"
    exit 1
fi

COMPOSE_CMD="docker-compose"
if ! command -v docker-compose &> /dev/null; then
    COMPOSE_CMD="docker compose"
fi

COMPOSE_FILE="docker-compose.test.yml"
SERVICE="bridge"

echo "1. Starting bridge container..."
$COMPOSE_CMD -f "$COMPOSE_FILE" up -d "$SERVICE"

echo "   Waiting for bridge to be healthy..."
for i in $(seq 1 30); do
    if $COMPOSE_CMD -f "$COMPOSE_FILE" ps "$SERVICE" 2>/dev/null | grep -q "healthy"; then
        echo "   Bridge is healthy."
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo "Error: Bridge did not become healthy in time"
        $COMPOSE_CMD -f "$COMPOSE_FILE" down 2>/dev/null || true
        exit 1
    fi
    sleep 1
done
echo ""

echo "2. Running Task 2.1 integration tests inside the bridge container..."
echo "   (Uses Task 1.9 tests: bridge accepts camera frames, port exposed.)"
$COMPOSE_CMD -f "$COMPOSE_FILE" exec -T -e BRIDGE_HOST=localhost "$SERVICE" \
    pytest tests/integration/test_task_1_9_docker.py -v

echo ""
echo "3. Stopping bridge..."
$COMPOSE_CMD -f "$COMPOSE_FILE" down 2>/dev/null || true

echo ""
echo "============================================="
echo "Task 2.1 Docker (bridge) test completed!"
echo "============================================="
