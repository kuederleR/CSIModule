#!/bin/bash

# --- Configuration Variables ---
CSI_RECEIVER_PATH="$HOME/esp/csi_projects/csi_receiver"
CSI_SENDER_PATH="$HOME/esp/csi_projects/csi_sender"
IDF_PATH="$HOME/esp/esp-idf" # Ensure this path is correct

# Function to flash a device
flash_device() {
    PROJECT_TYPE=$1      # 'receiver' or 'sender'
    CONFIG_NAME=$2       # e.g., 'ap_channel_6'
    PORT=$3              # e.g., /dev/ttyUSB0

    if [ "$PROJECT_TYPE" == "receiver" ]; then
        PROJECT_DIR=$CSI_RECEIVER_PATH
    elif [ "$PROJECT_TYPE" == "sender" ]; then
        PROJECT_DIR=$CSI_SENDER_PATH
    else
        echo "Error: Invalid project type '$PROJECT_TYPE'. Use 'receiver' or 'sender'."
        return 1
    fi

    CONFIG_FILE="$PROJECT_DIR/sdkconfig.defaults.$CONFIG_NAME"
    
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "Error: Configuration file $CONFIG_FILE not found."
        return 1
    fi

    echo "--- Preparing $PROJECT_TYPE with config $CONFIG_NAME on port $PORT ---"

    # 1. Clean up and set configuration
    cd "$PROJECT_DIR"
    rm -f sdkconfig
    cp "$CONFIG_FILE" "$PROJECT_DIR/sdkconfig.defaults"

    # 2. Build the project (includes reconfigure)
    $IDF_PATH/tools/idf.py build

    # 3. Flash the project and start monitor
    # The 'flash' command automatically uses the correct binaries from the 'build' folder.
    $IDF_PATH/tools/idf.py -p "$PORT" -b 921600 flash monitor
}

# --- Command Line Interface ---

if [ "$1" == "" ] || [ "$2" == "" ] || [ "$3" == "" ]; then
    echo "Usage: ./flash_csi.sh <type: receiver|sender> <config_name> <port>"
    echo "Example (Receiver): ./flash_csi.sh receiver ap_channel_6 /dev/ttyUSB0"
    echo "Example (Sender):   ./flash_csi.sh sender sta_connect_ap_1 /dev/ttyACM1"
    exit 1
fi

flash_device "$1" "$2" "$3"