#!/bin/bash

# Entrypoint script for ESP32 CSI flashing
# Receives: PORT TARGET MODE(rx|tx) [CHANNEL] [PMK] [MAC] [RATE]

set -e

# Source ESP-IDF environment
source /opt/esp/idf/export.sh

PORT=$1
TARGET=$2
MODE=$3
CHANNEL=${4:-11}  # Default to channel 11 if not provided
PMK=${5:-"pmk1234567890123"}  # Default PMK
MAC=${6:-"1a:00:00:00:00:00"}  # Default MAC
RATE=${7:-"MCS0_LGI"}  # Default rate

if [ -z "$PORT" ] || [ -z "$TARGET" ] || [ -z "$MODE" ]; then
    echo "Error: Missing required arguments"
    echo "Usage: <port> <target> <rx|tx> [channel] [pmk] [mac] [rate]"
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
echo "Configuring WiFi channel: $CHANNEL"
echo "Configuring ESP-NOW PMK: $PMK"
echo "Configuring sender MAC: $MAC"
echo "Configuring CSI rate: $RATE"
echo ""

# Convert MAC address format for C code (1a:00:00:00:00:00 -> 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00)
MAC_ARRAY=$(echo "$MAC" | sed 's/:/, 0x/g' | sed 's/^/0x/')

# Convert rate to proper constant (e.g., MCS0_LGI -> WIFI_PHY_RATE_MCS0_LGI)
if [[ "$RATE" == *"MCS"* ]]; then
    RATE_CONSTANT="WIFI_PHY_RATE_$RATE"
else
    RATE_CONSTANT="$RATE"
fi

# Configure the parameters in the source code before building
if [ -f "main/app_main.c" ]; then
    echo "Updating configuration in source code..."
    
    # Update channel
    sed -i "s/#define CONFIG_LESS_INTERFERENCE_CHANNEL.*/#define CONFIG_LESS_INTERFERENCE_CHANNEL   $CHANNEL/" main/app_main.c
    
    # Update PMK
    sed -i "s/esp_now_set_pmk((uint8_t *)\".*\");/esp_now_set_pmk((uint8_t *)\"$PMK\");/" main/app_main.c
    
    # Update MAC address
    sed -i "s/CONFIG_CSI_SEND_MAC\[\] = {.*};/CONFIG_CSI_SEND_MAC[] = {$MAC_ARRAY};/" main/app_main.c
    
    # Update rate
    sed -i "s/#define CONFIG_ESP_NOW_RATE.*/#define CONFIG_ESP_NOW_RATE             $RATE_CONSTANT/" main/app_main.c
    
    echo "Configuration updated:"
    echo "  Channel: $CHANNEL"
    echo "  PMK: $PMK"
    echo "  MAC: $MAC ($MAC_ARRAY)"
    echo "  Rate: $RATE_CONSTANT"
else
    echo "Warning: app_main.c not found, using default configuration"
fi

echo ""

# Set the target
idf.py set-target "$TARGET"

# Build the project with the new channel configuration
echo ""
echo "Building project with channel $CHANNEL..."
idf.py build

echo ""
echo "Flashing to port: $PORT"
echo ""

# Flash the device
idf.py flash -b 921600 -p "$PORT"

echo ""
echo "Flashing complete!"
echo "Configuration applied:"
echo "  WiFi channel: $CHANNEL"
echo "  ESP-NOW PMK: $PMK"
echo "  Sender MAC: $MAC"
echo "  CSI rate: $RATE_CONSTANT"
