#!/bin/bash

# ESP32 CSI Flash Tool - Docker Interface
# Flash ESP32 devices with specified target and port

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Docker image name
IMAGE_NAME="esp32-csi-flash"

print_usage() {
    echo "ESP32 CSI Flash Tool - Docker Interface"
    echo ""
    echo "Usage: $0 <port> <target> <rx|tx>"
    echo ""
    echo "Arguments:"
    echo "  port    - Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)"
    echo "  target  - ESP32 target (e.g., esp32c6, esp32s3, esp32)"
    echo "  rx|tx   - Device type: 'rx' for receiver, 'tx' for sender"
    echo ""
    echo "Examples:"
    echo "  $0 /dev/ttyACM0 esp32c6 rx"
    echo "  $0 /dev/ttyUSB0 esp32s3 tx"
    echo ""
}

check_docker() {
    if ! command -v docker &> /dev/null; then
        echo "Error: Docker is not installed or not in PATH"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        echo "Error: Docker daemon is not running or user lacks permissions"
        echo "Try: sudo usermod -aG docker \$USER && newgrp docker"
        exit 1
    fi
}

# Check arguments
if [ "$#" -ne 3 ]; then
    echo "Error: Exactly 3 arguments required"
    echo ""
    print_usage
    exit 1
fi

PORT=$1
TARGET=$2
MODE=$3

# Validate mode
if [ "$MODE" != "rx" ] && [ "$MODE" != "tx" ]; then
    echo "Error: Third argument must be 'rx' or 'tx'"
    echo ""
    print_usage
    exit 1
fi

# Validate port exists
if [ ! -e "$PORT" ]; then
    echo "Error: Port $PORT does not exist"
    exit 1
fi

check_docker

# Check if Docker image exists, build if it doesn't
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Docker image '$IMAGE_NAME' not found. Building it now..."
    echo ""
    docker build -t "$IMAGE_NAME" .
    echo ""
    echo "Docker image built successfully!"
    echo ""
fi

echo "Flashing ESP32 $MODE device..."
echo "  Port: $PORT"
echo "  Target: $TARGET"
echo "  Mode: $MODE"
echo ""

docker run --rm --privileged \
    -v /dev:/dev \
    "$IMAGE_NAME" "$PORT" "$TARGET" "$MODE"