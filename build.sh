#!/bin/bash
#
# build.sh - Build and optionally push Docker image for the mavsim bridge
# (user_repo_new)
#
# Usage:
#   ./build.sh [--push] [--tag TAG]
#
# Examples:
#   ./build.sh                    # Build image locally
#   ./build.sh --push             # Build and push to Docker Hub
#   ./build.sh --tag v1.0         # Build with specific tag
#   ./build.sh --tag v1.0 --push  # Build, tag, and push
#

set -e  # Exit on error

# Docker image configuration
IMAGE_NAME="mavlab/mavsim-controller"
DEFAULT_TAG="latest"
TAG="${DEFAULT_TAG}"
PUSH=false

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --push)
            PUSH=true
            shift
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        *)
            print_warn "Unknown option: $1"
            shift
            ;;
    esac
done

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Get the directory where this script is located - which is now the repository
# root. This read `ROOT_DIR="$SCRIPT_DIR/.."` while this lived in the mavsim
# repo as user_repo_new/, because the build context had to be mavsim's root to
# reach sensor_bridge/ and ros2_ws/src/interfaces/. Both now live in this repo,
# so the context is simply here - and leaving the `..` in place would silently
# hand Docker the parent directory of the whole checkout as build context.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"

# Build for multiple platforms (Linux AMD64, ARM64)
# Note: Multi-platform builds require Docker Buildx
print_info "Building Docker image: ${IMAGE_NAME}:${TAG}"
print_info "Build context: ${ROOT_DIR} (repository root)"
print_info "Dockerfile: ${SCRIPT_DIR}/Dockerfile"

# Check if buildx is available
if docker buildx version &> /dev/null; then
    print_info "Using Docker Buildx for multi-platform build"

    # Create builder instance if it doesn't exist
    if ! docker buildx ls | grep -q mavsim-builder; then
        print_info "Creating buildx builder instance..."
        docker buildx create --name mavsim-builder --use || true
        docker buildx inspect --bootstrap || true
    fi

    # Build for multiple platforms
    PLATFORMS="linux/amd64,linux/arm64"
    print_info "Building for platforms: ${PLATFORMS}"

    if [ "$PUSH" = true ]; then
        docker buildx build \
            --platform "${PLATFORMS}" \
            --tag "${IMAGE_NAME}:${TAG}" \
            --push \
            -f "${SCRIPT_DIR}/Dockerfile" \
            "${ROOT_DIR}"
        print_info "Image built and pushed: ${IMAGE_NAME}:${TAG}"
    else
        # For local build without push, build for current platform only
        docker buildx build \
            --platform "${PLATFORMS}" \
            --tag "${IMAGE_NAME}:${TAG}" \
            --load \
            -f "${SCRIPT_DIR}/Dockerfile" \
            "${ROOT_DIR}" || {
            print_warn "Multi-platform build with --load failed, trying single platform..."
            docker build \
                --tag "${IMAGE_NAME}:${TAG}" \
                -f "${SCRIPT_DIR}/Dockerfile" \
                "${ROOT_DIR}"
        }
        print_info "Image built locally: ${IMAGE_NAME}:${TAG}"
    fi
else
    print_warn "Docker Buildx not available, building for current platform only"
    print_warn "For multi-platform builds, install Docker Buildx: https://docs.docker.com/buildx/working-with-buildx/"

    docker build \
        --tag "${IMAGE_NAME}:${TAG}" \
        -f "${SCRIPT_DIR}/Dockerfile" \
        "${ROOT_DIR}"

    if [ "$PUSH" = true ]; then
        print_info "Pushing image to Docker Hub..."
        docker push "${IMAGE_NAME}:${TAG}"
        print_info "Image pushed: ${IMAGE_NAME}:${TAG}"
    else
        print_info "Image built locally: ${IMAGE_NAME}:${TAG}"
    fi
fi

print_info "Build complete!"
print_info "To test the image:"
print_info "  docker run --rm ${IMAGE_NAME}:${TAG} --help"

