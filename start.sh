#!/bin/bash
#
# start.sh - Run the mavsim ROS2 bridge controller using the published image
#
# The bridge connects to the simulation and exposes it as LOCAL ROS2 topics:
#   - publishes telemetry:  /<vessel>/vessel_state, /vessel_state_der,
#                           /odometry_sim (+ sensors with --enable-sensors)
#   - subscribes commands:  /<vessel>/actuator_cmd   (interfaces/Actuator)
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
#              [--enable-sensors] [--rate HZ] [--ros-domain-id N]
#              [--cmd-timeout SEC] [--observe-others]
#   ./start.sh --token /path/to/token.json [options...]
#   ./start.sh --mode web [--port 8888] [--backend-url URL] [--ros-domain-id N]
#
# Examples:
#   ./start.sh ABC123
#   ./start.sh ABC123 --enable-sensors --observe-others
#   ./start.sh ABC123 --ros-domain-id 42
#   ./start.sh --token ./mavsim_token_abc12345.json --observe-others
#   ./start.sh --mode web
#
# Recordings are saved to ./recordings/ on your machine.
#

set -e

DOCKER_IMAGE="mavlab/mavsim-controller:latest"
CONTAINER_NAME="mavsim-bridge-$$"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DEFAULT_BACKEND_URL="http://localhost:5000"
DEFAULT_WEBAPP_PORT=8888
DEFAULT_ROS_DOMAIN_ID=0
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

# ===========================================================
# Web mode
# ===========================================================
if [ "$MODE" = "web" ]; then
    BACKEND_URL="$DEFAULT_BACKEND_URL"
    while [[ $# -gt 0 ]]; do
        case $1 in
            --port)          WEBAPP_PORT="$2"; shift 2 ;;
            --backend-url)   BACKEND_URL="$2"; shift 2 ;;
            --ros-domain-id) ROS_DOMAIN_ID_VAL="$2"; shift 2 ;;
            *)               print_warn "Unknown argument: $1 (ignored)"; shift ;;
        esac
    done

    if [ ! -f "$WEBAPP_FILE" ]; then
        print_error "bridge_webapp.py not found next to start.sh ($WEBAPP_FILE)"
        exit 1
    fi

    CONTAINER_BACKEND_URL="$(rewrite_url "$BACKEND_URL")"

    print_info "Starting mavsim bridge in WEB mode"
    print_info "  Image:        $DOCKER_IMAGE"
    print_info "  Web UI:       http://localhost:$WEBAPP_PORT"
    print_info "  Backend:      $BACKEND_URL"
    [ "$BACKEND_URL" != "$CONTAINER_BACKEND_URL" ] && \
        print_info "  (inside container: $CONTAINER_BACKEND_URL)"
    print_info "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VAL"
    print_info "  Recordings:    $RECORDINGS_DIR"
    echo ""

    DOCKER_ARGS=(docker run --rm --name "$CONTAINER_NAME")

    if [ "$USE_HOST_NET" = true ]; then
        DOCKER_ARGS+=(--network host)
    else
        DOCKER_ARGS+=(-p "$WEBAPP_PORT:$WEBAPP_PORT" -p "7001-7095:7001-7095")
    fi

    # The bridge webapp is mounted from this folder and launched via a custom
    # command, so no image rebuild is needed. run_controller.py (in /app)
    # discovers the mounted bridge as /app/user_code/my_controller.py.
    WEB_CMD="source /opt/ros/humble/setup.bash \
        && source /ros2_ws/install/setup.bash \
        && cd /app \
        && exec python3 /app/user_code/bridge_webapp.py --port '$WEBAPP_PORT' --backend-url '$CONTAINER_BACKEND_URL'"

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
    echo "  $0 <controller-code> [--vessel-name NAME] [--backend-url URL] [--enable-sensors] [--rate HZ] [--ros-domain-id N] [--cmd-timeout SEC] [--observe-others]"
    echo "  $0 --token /path/to/token.json [options...]"
    echo "  $0 --mode web [--port 8888] [--backend-url URL] [--ros-domain-id N]"
    echo ""
    echo "Examples:"
    echo "  $0 ABC123"
    echo "  $0 ABC123 --enable-sensors --observe-others"
    echo "  $0 --token ./mavsim_token_abc12345.json"
    echo "  $0 --mode web"
    exit 1
fi

CONTROLLER_CODE=""
if [ -z "$TOKEN_FILE" ]; then
    CONTROLLER_CODE="$1"; shift
fi

BACKEND_URL="$DEFAULT_BACKEND_URL"
VESSEL_NAME=""
ENABLE_SENSORS=false
OBSERVE_OTHERS=false
CMD_TIMEOUT="$DEFAULT_CMD_TIMEOUT"
# Args understood by the container's run_controller.py.
PYTHON_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        --backend-url)     BACKEND_URL="$2"; shift 2 ;;
        --vessel-name)     VESSEL_NAME="$2"; PYTHON_ARGS+=("--vessel-name" "$2"); shift 2 ;;
        --enable-sensors)  ENABLE_SENSORS=true; PYTHON_ARGS+=("--enable-sensors"); shift ;;
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
print_info "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VAL"
print_info "  Cmd timeout:  ${CMD_TIMEOUT}s"
print_info "  Recordings:   $RECORDINGS_DIR"
[ -n "$VESSEL_NAME" ]        && print_info "  Vessel:       $VESSEL_NAME"
[ "$ENABLE_SENSORS" = true ] && print_info "  Sensors:      enabled"
[ "$ENABLE_SENSORS" = true ] && print_info "  Camera viewer: open camera_viewer.html in a browser"
[ "$OBSERVE_OTHERS" = true ] && print_info "  Observe-others: enabled (read-only telemetry for non-owned vessels)"
echo ""

DOCKER_ARGS=(docker run --rm --name "$CONTAINER_NAME")

if [ "$USE_HOST_NET" = true ]; then
    DOCKER_ARGS+=(--network host)
elif [ "$ENABLE_SENSORS" = true ]; then
    DOCKER_ARGS+=(-p "7001-7095:7001-7095")
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
