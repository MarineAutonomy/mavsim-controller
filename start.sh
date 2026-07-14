#!/bin/bash
#
# start.sh - Run the mavsim ROS2 bridge controller using the published image
#
# The bridge connects to the simulation and exposes it as LOCAL ROS2 topics:
#   - publishes telemetry:  /<vessel>/vessel_state, /vessel_state_der,
#                           /odometry_sim, plus camera/lidar/imu/gps/encoder
#                           topics (sensors are always enabled - see
#                           plans/plan_headless_observer.md)
#   - subscribes commands:  /<vessel>/actuator_cmd   (interfaces/Actuator)
#
# Camera/lidar data is captured in a browser (WebGL rendering), so a headless
# Chromium tab is launched automatically alongside the bridge to trigger that
# rendering without needing a human to keep a tab open - it needs to know
# where the MAVSim frontend is actually hosted via --frontend-url (defaults
# to http://localhost:5173, override whenever the bridge isn't on the same
# machine as the frontend - e.g. a remote/lab server, or once deployed to AWS).
#
# You then run YOUR controller (any language) so that it subscribes to the
# telemetry topics and publishes interfaces/Actuator on /<vessel>/actuator_cmd.
# The bridge forwards the latest command to the simulator over REST.
#
# Modes:
#   CLI mode (default): ./start.sh <controller-code> [options...]
#   Token mode:         ./start.sh --token /path/to/token.json [options...]
#   Web mode:           ./start.sh --mode web [options...]
#
# Usage:
#   ./start.sh <controller-code> [--vessel-name NAME] [--backend-url URL]
#              [--frontend-url URL] [--rate HZ] [--ros-domain-id N]
#              [--cmd-timeout SEC] [--observe-others]
#   ./start.sh --token /path/to/token.json [options...]
#   ./start.sh --mode web [--port 8888] [--backend-url URL] [--frontend-url URL] [--ros-domain-id N]
#
# Examples:
#   ./start.sh ABC123
#   ./start.sh ABC123 --observe-others
#   ./start.sh ABC123 --frontend-url http://my-server:5173
#   ./start.sh --token ./mavsim_token_abc12345.json --observe-others
#   ./start.sh --mode web
#   ./start.sh ABC123 --ros-domain-id 43   # run a second bridge alongside this one
#
# --ros-domain-id defaults to 42 (not 0) on every run, including local Docker
# testing against a `docker compose` mavsim stack - see the comment by
# DEFAULT_ROS_DOMAIN_ID below for why.
#
# Recordings are saved to ./recordings/ on your machine.
#

set -e

DOCKER_IMAGE="mavlab/mavsim-controller:latest"
CONTAINER_NAME="mavsim-bridge-$$"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DEFAULT_BACKEND_URL="http://localhost:5000"
DEFAULT_FRONTEND_URL="http://localhost:5173"
DEFAULT_WEBAPP_PORT=8888
# Deliberately NOT 0. A local `docker compose` mavsim stack assigns each
# simulation session its own ROS_DOMAIN_ID starting at 0 (one per session
# slot, MAX_SESSIONS=5 by default -> domains 0-4, see
# simulation_task/app/services/session_manager.py). ROS2/DDS discovery is
# network-scoped, not container-scoped - if this bridge defaulted to the same
# domain as a local session, it would silently see (and could record) that
# session's own internal ROS2 topics via DDS, not just its own. 42 is always
# set here, on every local Docker run, specifically to stay clear of that
# range. Irrelevant for real cloud deployments (bridge and simulation task
# are on separate networks there), but always set anyway since this same
# start.sh is what people use for local testing too.
DEFAULT_ROS_DOMAIN_ID=42
DEFAULT_CMD_TIMEOUT=1.0
RECORDINGS_DIR="$SCRIPT_DIR/recordings"
BRIDGE_FILE="$SCRIPT_DIR/bridge_controller.py"
WEBAPP_FILE="$SCRIPT_DIR/bridge_webapp.py"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ---- Detect mode ----
MODE="cli"
WEBAPP_PORT="$DEFAULT_WEBAPP_PORT"
ROS_DOMAIN_ID_VAL="$DEFAULT_ROS_DOMAIN_ID"

if [ $# -ge 1 ] && [ "$1" = "--mode" ]; then
    [ $# -lt 2 ] && { print_error "--mode requires a value: web or cli"; exit 1; }
    MODE="$2"; shift 2
    [[ "$MODE" != "web" && "$MODE" != "cli" ]] && { print_error "Invalid mode: $MODE"; exit 1; }
fi

# ---- Prerequisites ----
mkdir -p "$RECORDINGS_DIR"

if ! command -v docker &>/dev/null; then
    print_error "Docker is not installed. See https://docs.docker.com/get-docker/"
    exit 1
fi

if [ ! -f "$BRIDGE_FILE" ]; then
    print_error "bridge_controller.py not found next to start.sh ($BRIDGE_FILE)"
    print_error "Re-clone user_repo_new or restore bridge_controller.py."
    exit 1
fi

# Pull image if not present locally
if ! docker image inspect "$DOCKER_IMAGE" &>/dev/null; then
    print_info "Pulling $DOCKER_IMAGE ..."
    docker pull "$DOCKER_IMAGE" || { print_error "Failed to pull image"; exit 1; }
fi

# ---- Network mode ----
# --network host works only on Linux and is what lets your own ROS2 code on the
# host talk to the bridge's local topics. On macOS / Windows Docker Desktop it
# is ignored, so use the sibling-container path (see examples/docker/).
USE_HOST_NET=true
if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
    USE_HOST_NET=false
fi

rewrite_url() {
    local url="$1"
    if [ "$USE_HOST_NET" = false ]; then
        url="${url//localhost/host.docker.internal}"
        url="${url//127.0.0.1/host.docker.internal}"
    fi
    echo "$url"
}

# ---- GPU passthrough (NVIDIA only for now) ----
# The headless sensor observer defaults to CPU-only SwiftShader rendering,
# which can become unresponsive under real load (multiple vessels/sensors,
# heavy post-processing). When a real GPU is available, request passthrough
# so the observer can use it instead - examples/observer.py independently
# re-checks GPU access from inside the container before picking Chromium's
# render flags, so a false positive here just falls back to SwiftShader
# rather than breaking anything. Detection has to happen here (not inside
# the container) because Docker only grants GPU access at container-creation
# time via `--gpus`, and passing that flag unconditionally would hard-fail
# `docker run` on any machine without the nvidia-container-toolkit installed
# at all - not degrade gracefully.
GPU_AVAILABLE=false
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1 \
   && command -v docker >/dev/null 2>&1 && docker info 2>/dev/null | grep -q '^ Runtimes:.*nvidia'; then
    GPU_AVAILABLE=true
fi

# ===========================================================
# Web mode
# ===========================================================
if [ "$MODE" = "web" ]; then
    BACKEND_URL="$DEFAULT_BACKEND_URL"
    FRONTEND_URL="$DEFAULT_FRONTEND_URL"
    while [[ $# -gt 0 ]]; do
        case $1 in
            --port)          WEBAPP_PORT="$2"; shift 2 ;;
            --backend-url)   BACKEND_URL="$2"; shift 2 ;;
            --frontend-url)  FRONTEND_URL="$2"; shift 2 ;;
            --ros-domain-id) ROS_DOMAIN_ID_VAL="$2"; shift 2 ;;
            *)               print_warn "Unknown argument: $1 (ignored)"; shift ;;
        esac
    done

    if [ ! -f "$WEBAPP_FILE" ]; then
        print_error "bridge_webapp.py not found next to start.sh ($WEBAPP_FILE)"
        exit 1
    fi

    CONTAINER_BACKEND_URL="$(rewrite_url "$BACKEND_URL")"
    CONTAINER_FRONTEND_URL="$(rewrite_url "$FRONTEND_URL")"

    print_info "Starting mavsim bridge in WEB mode"
    print_info "  Image:        $DOCKER_IMAGE"
    print_info "  Web UI:       http://localhost:$WEBAPP_PORT"
    print_info "  Backend:      $BACKEND_URL"
    [ "$BACKEND_URL" != "$CONTAINER_BACKEND_URL" ] && \
        print_info "  (inside container: $CONTAINER_BACKEND_URL)"
    print_info "  Frontend:     $FRONTEND_URL"
    [ "$FRONTEND_URL" != "$CONTAINER_FRONTEND_URL" ] && \
        print_info "  (inside container: $CONTAINER_FRONTEND_URL)"
    print_info "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VAL"
    print_info "  Recordings:    $RECORDINGS_DIR"
    print_info "  Visualizer:    http://localhost:8899 (time histories, camera, point cloud, overlay)"
    if [ "$GPU_AVAILABLE" = true ]; then
        print_info "  GPU:           NVIDIA GPU detected - passing through for hardware-accelerated rendering"
    else
        print_info "  GPU:           none detected - observer uses CPU-only SwiftShader rendering"
    fi
    echo ""

    DOCKER_ARGS=(docker run --rm --name "$CONTAINER_NAME")
    [ "$GPU_AVAILABLE" = true ] && DOCKER_ARGS+=(--gpus all)

    if [ "$USE_HOST_NET" = true ]; then
        DOCKER_ARGS+=(--network host)
    else
        DOCKER_ARGS+=(-p "$WEBAPP_PORT:$WEBAPP_PORT" -p "7001-7095:7001-7095" -p "9090:9090" -p "8899:8899")
    fi

    # The bridge webapp is mounted from this folder and launched via a custom
    # command, so no image rebuild is needed. run_controller.py (in /app)
    # discovers the mounted bridge as /app/user_code/my_controller.py.
    WEB_CMD="source /opt/ros/humble/setup.bash \
        && source /ros2_ws/install/setup.bash \
        && cd /app \
        && exec python3 /app/user_code/bridge_webapp.py --port '$WEBAPP_PORT' --backend-url '$CONTAINER_BACKEND_URL' --frontend-url '$CONTAINER_FRONTEND_URL'"

    DOCKER_ARGS+=(
        -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VAL"
        -v "$RECORDINGS_DIR:/tmp/mavsim_bags"
        -v "$BRIDGE_FILE:/app/user_code/my_controller.py:ro"
        -v "$WEBAPP_FILE:/app/user_code/bridge_webapp.py:ro"
        --entrypoint /bin/bash
        "$DOCKER_IMAGE"
        -c "$WEB_CMD"
    )

    exec "${DOCKER_ARGS[@]}"
fi

# ===========================================================
# CLI / Token mode
# ===========================================================

# Check for --token mode first
TOKEN_FILE=""
if [ $# -ge 1 ] && [ "$1" = "--token" ]; then
    if [ $# -lt 2 ]; then
        print_error "--token requires a path to a JSON token file"
        exit 1
    fi
    TOKEN_FILE="$2"
    shift 2
    if [ ! -f "$TOKEN_FILE" ]; then
        print_error "Token file not found: $TOKEN_FILE"
        exit 1
    fi
fi

if [ -z "$TOKEN_FILE" ] && [ $# -lt 1 ]; then
    print_error "Controller code or --token is required in CLI mode"
    echo ""
    echo "Usage:"
    echo "  $0 <controller-code> [--vessel-name NAME] [--backend-url URL] [--frontend-url URL] [--rate HZ] [--ros-domain-id N] [--cmd-timeout SEC] [--observe-others]"
    echo "  $0 --token /path/to/token.json [options...]"
    echo "  $0 --mode web [--port 8888] [--backend-url URL] [--frontend-url URL] [--ros-domain-id N]"
    echo ""
    echo "Examples:"
    echo "  $0 ABC123"
    echo "  $0 ABC123 --observe-others"
    echo "  $0 ABC123 --frontend-url http://my-server:5173"
    echo "  $0 --token ./mavsim_token_abc12345.json"
    echo "  $0 --mode web"
    exit 1
fi

CONTROLLER_CODE=""
if [ -z "$TOKEN_FILE" ]; then
    CONTROLLER_CODE="$1"; shift
fi

BACKEND_URL="$DEFAULT_BACKEND_URL"
FRONTEND_URL="$DEFAULT_FRONTEND_URL"
VESSEL_NAME=""
OBSERVE_OTHERS=false
CMD_TIMEOUT="$DEFAULT_CMD_TIMEOUT"
# Args understood by the container's run_controller.py.
PYTHON_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        --backend-url)     BACKEND_URL="$2"; shift 2 ;;
        --frontend-url)    FRONTEND_URL="$2"; shift 2 ;;
        --vessel-name)     VESSEL_NAME="$2"; PYTHON_ARGS+=("--vessel-name" "$2"); shift 2 ;;
        --rate)            PYTHON_ARGS+=("--rate" "$2"); shift 2 ;;
        # Bridge-only options -> passed to the container via environment vars,
        # since run_controller.py does not understand them.
        --ros-domain-id)   ROS_DOMAIN_ID_VAL="$2"; shift 2 ;;
        --cmd-timeout)     CMD_TIMEOUT="$2"; shift 2 ;;
        --observe-others)  OBSERVE_OTHERS=true; shift ;;
        *)                 PYTHON_ARGS+=("$1"); shift ;;
    esac
done

CONTAINER_BACKEND_URL="$(rewrite_url "$BACKEND_URL")"
CONTAINER_FRONTEND_URL="$(rewrite_url "$FRONTEND_URL")"
PYTHON_ARGS+=("--frontend-url" "$CONTAINER_FRONTEND_URL")

if [ -n "$TOKEN_FILE" ]; then
    print_info "Starting mavsim bridge in TOKEN mode"
    print_info "  Image:        $DOCKER_IMAGE"
    print_info "  Token:        $TOKEN_FILE"
else
    print_info "Starting mavsim bridge in CLI mode"
    print_info "  Image:        $DOCKER_IMAGE"
    print_info "  Code:         $CONTROLLER_CODE"
fi
print_info "  Backend:      $BACKEND_URL"
[ "$BACKEND_URL" != "$CONTAINER_BACKEND_URL" ] && \
    print_info "  (inside container: $CONTAINER_BACKEND_URL)"
print_info "  Frontend:     $FRONTEND_URL"
[ "$FRONTEND_URL" != "$CONTAINER_FRONTEND_URL" ] && \
    print_info "  (inside container: $CONTAINER_FRONTEND_URL)"
print_info "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VAL"
print_info "  Cmd timeout:  ${CMD_TIMEOUT}s"
print_info "  Recordings:   $RECORDINGS_DIR"
[ -n "$VESSEL_NAME" ]        && print_info "  Vessel:       $VESSEL_NAME"
print_info "  Sensors:      always enabled (camera/lidar via headless observer)"
print_info "  Visualizer:   http://localhost:8899 (time histories, camera, point cloud, overlay)"
if [ "$GPU_AVAILABLE" = true ]; then
    print_info "  GPU:          NVIDIA GPU detected - passing through for hardware-accelerated rendering"
else
    print_info "  GPU:          none detected - observer uses CPU-only SwiftShader rendering"
fi
[ "$OBSERVE_OTHERS" = true ] && print_info "  Observe-others: enabled (read-only telemetry for non-owned vessels)"
echo ""

DOCKER_ARGS=(docker run --rm --name "$CONTAINER_NAME")
[ "$GPU_AVAILABLE" = true ] && DOCKER_ARGS+=(--gpus all)

if [ "$USE_HOST_NET" = true ]; then
    DOCKER_ARGS+=(--network host)
else
    # Sensors (camera/lidar) are always enabled now, so these ports are always needed.
    # 9090/8899: rosbridge websocket + the local ROS2 topic visualizer's Flask app.
    DOCKER_ARGS+=(-p "7001-7095:7001-7095" -p "9090:9090" -p "8899:8899")
fi

# Bridge configuration via environment variables.
DOCKER_ARGS+=(-e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VAL")
DOCKER_ARGS+=(-e "MAVSIM_CMD_TIMEOUT=$CMD_TIMEOUT")
[ "$OBSERVE_OTHERS" = true ] && DOCKER_ARGS+=(-e "MAVSIM_OBSERVE_OTHERS=1")

DOCKER_ARGS+=(
    -v "$RECORDINGS_DIR:/tmp/mavsim_bags"
    -v "$BRIDGE_FILE:/app/user_code/my_controller.py:ro"
)

if [ -n "$TOKEN_FILE" ]; then
    DOCKER_ARGS+=(
        -v "$(cd "$(dirname "$TOKEN_FILE")" && pwd)/$(basename "$TOKEN_FILE"):/app/token.json:ro"
        "$DOCKER_IMAGE"
        --token /app/token.json
        "${PYTHON_ARGS[@]}"
    )
else
    DOCKER_ARGS+=(
        "$DOCKER_IMAGE"
        --code "$CONTROLLER_CODE"
        --backend-url "$CONTAINER_BACKEND_URL"
        "${PYTHON_ARGS[@]}"
    )
fi

exec "${DOCKER_ARGS[@]}"
