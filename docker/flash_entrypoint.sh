#!/bin/bash

# Entrypoint script for ESP32 CSI flashing
# Receives: PORT TARGET MODE(rx|tx)

set -e

# Source ESP-IDF environment
source /opt/esp/idf/export.sh

PORT=$1
TARGET=$2
MODE=$3

if [ -z "$PORT" ] || [ -z "$TARGET" ] || [ -z "$MODE" ]; then
    echo "Error: Missing required arguments"
    echo "Usage: <port> <target> <rx|tx>"
    exit 1
fi

# Determine directory based on mode
if [ "$MODE" = "rx" ]; then
    WORK_DIR="/workspace/esp/csi_recv"
elif [ "$MODE" = "tx" ]; then
    WORK_DIR="/workspace/esp/csi_send"
else
    echo "Error: Mode must be 'rx' or 'tx', got: $MODE"
    exit 1
fi

# Change to the appropriate directory
cd "$WORK_DIR"

echo "Working directory: $WORK_DIR"
echo "Setting target to: $TARGET"
echo ""

# Set the target
idf.py set-target "$TARGET"

echo ""
echo "Flashing to port: $PORT"
echo ""

# Flash the device
idf.py flash -b 921600 -p "$PORT"

echo ""
echo "Flashing complete!"
