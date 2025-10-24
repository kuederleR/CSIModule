#!/bin/bash

# ESP32 CSI Flash Tool - Docker Interface
# Flash ESP32 devices with specified target and port
#
# TO ADD NEW ARGUMENTS:
# 1. Add to ARGS array: ["new_arg"]=""
# 2. Add to ARG_DESCRIPTIONS: ["new_arg"]="Description of the new argument"
# 3. Add to REQUIRED_ARGS if required: REQUIRED_ARGS=("port" "target" "mode" "new_arg")
# 4. Add case in parse_arguments: --new-arg) ARGS["new_arg"]="$2"; shift 2 ;;
# 5. Add validation function if needed: validate_new_arg() { ... }
# 6. Call validation in validate_arguments(): validate_new_arg "${ARGS[new_arg]}" || exit 1
# 7. Update docker command to pass new argument
# 8. Update flash_entrypoint.sh to handle the new parameter

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Docker image name
IMAGE_NAME="esp32-csi-flash"

print_usage() {
    echo "ESP32 CSI Flash Tool - Docker Interface"
    echo ""
    echo "Usage:"
    echo "  $0 <port> <target> <mode>                    # Positional arguments"
    echo "  $0 --port <port> --target <target> --mode <mode> [--channel <ch>]  # Named arguments"
    echo ""
    echo "Required Arguments:"
    for arg in "${REQUIRED_ARGS[@]}"; do
        printf "  %-8s - %s\n" "$arg" "${ARG_DESCRIPTIONS[$arg]}"
    done
    echo ""
    echo "Optional Arguments:"
    echo "  channel  - ${ARG_DESCRIPTIONS[channel]} (default: ${ARGS[channel]})"
    echo "  pmk      - ${ARG_DESCRIPTIONS[pmk]} (default: ${ARGS[pmk]})"
    echo "  mac      - ${ARG_DESCRIPTIONS[mac]} (default: ${ARGS[mac]})"
    echo "  rate     - ${ARG_DESCRIPTIONS[rate]} (default: ${ARGS[rate]})"
    echo ""
    echo "Examples:"
    echo "  $0 /dev/ttyACM0 esp32c6 rx"
    echo "  $0 /dev/ttyUSB0 esp32s3 tx"
    echo "  $0 --port /dev/ttyACM0 --target esp32c6 --mode rx"
    echo "  $0 --target esp32s3 --port /dev/ttyUSB0 --mode tx"
    echo "  $0 --port /dev/ttyACM0 --target esp32c6 --mode rx --channel 6"
    echo "  $0 --port /dev/ttyUSB0 --target esp32s3 --mode tx --pmk mypassword123"
    echo "  $0 --port /dev/ttyACM0 --target esp32c6 --mode rx --mac 2a:01:02:03:04:05"
    echo "  $0 --port /dev/ttyUSB0 --target esp32s3 --mode tx --rate MCS7_LGI"
    echo "  $0 /dev/ttyUSB0 esp32s3 tx  # (uses all default values)"
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

# Configuration for arguments - easily extensible
declare -A ARGS=(
    ["port"]=""
    ["target"]=""
    ["mode"]=""
    ["channel"]="11"
    ["pmk"]="pmk1234567890123"
    ["mac"]="1a:00:00:00:00:00"
    ["rate"]="MCS0_LGI"
)

declare -A ARG_DESCRIPTIONS=(
    ["port"]="Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)"
    ["target"]="ESP32 target (e.g., esp32c6, esp32s3, esp32)"
    ["mode"]="Device type: 'rx' for receiver, 'tx' for sender"
    ["channel"]="Communication channel (e.g., 6, 11)"
    ["pmk"]="ESP-NOW primary master key/password (16 chars)"
    ["mac"]="Sender MAC address (e.g., 1a:00:00:00:00:00)"
    ["rate"]="CSI transmission rate (e.g., MCS0_LGI, MCS1_LGI, MCS7_LGI)"
)

# Define required arguments in order for positional parsing
REQUIRED_ARGS=("port" "target" "mode")

# Validation functions for arguments
validate_mode() {
    local mode="$1"
    if [ "$mode" != "rx" ] && [ "$mode" != "tx" ]; then
        echo "Error: Mode must be 'rx' or 'tx', got '$mode'"
        return 1
    fi
    return 0
}

validate_port() {
    local port="$1"
    if [ ! -e "$port" ]; then
        echo "Error: Port $port does not exist"
        return 1
    fi
    return 0
}

validate_mac() {
    local mac="$1"
    if [[ ! "$mac" =~ ^[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}$ ]]; then
        echo "Error: Invalid MAC address format '$mac'. Expected format: xx:xx:xx:xx:xx:xx"
        return 1
    fi
    return 0
}

validate_pmk() {
    local pmk="$1"
    if [ ${#pmk} -lt 8 ] || [ ${#pmk} -gt 64 ]; then
        echo "Error: PMK must be between 8 and 64 characters, got ${#pmk} characters"
        return 1
    fi
    return 0
}

validate_rate() {
    local rate="$1"
    # Common ESP-NOW rates
    case "$rate" in
        MCS0_LGI|MCS1_LGI|MCS2_LGI|MCS3_LGI|MCS4_LGI|MCS5_LGI|MCS6_LGI|MCS7_LGI|\
        MCS0_SGI|MCS1_SGI|MCS2_SGI|MCS3_SGI|MCS4_SGI|MCS5_SGI|MCS6_SGI|MCS7_SGI|\
        WIFI_PHY_RATE_*)
            return 0
            ;;
        *)
            echo "Error: Invalid rate '$rate'. Common values: MCS0_LGI, MCS1_LGI, ..., MCS7_LGI, MCS0_SGI, ..., MCS7_SGI"
            return 1
            ;;
    esac
}

# Add validation function calls here for easy extension
validate_arguments() {
    validate_mode "${ARGS[mode]}" || exit 1
    validate_mac "${ARGS[mac]}" || exit 1
    validate_pmk "${ARGS[pmk]}" || exit 1
    validate_rate "${ARGS[rate]}" || exit 1
    validate_port "${ARGS[port]}" || exit 1  # Check port last since it accesses filesystem
    # Add more validation calls here as needed
}

parse_arguments() {
    # Check if using named arguments (starts with --)
    if [[ "$1" == --* ]] || [[ "$1" == "-h" ]]; then
        # Parse named arguments
        while [[ $# -gt 0 ]]; do
            case $1 in
                --port)
                    ARGS["port"]="$2"
                    shift 2
                    ;;
                --target)
                    ARGS["target"]="$2"
                    shift 2
                    ;;
                --mode)
                    ARGS["mode"]="$2"
                    shift 2
                    ;;
                # Add new arguments here - follow the pattern:
                # --new-arg)
                #     ARGS["new_arg"]="$2"
                #     shift 2
                #     ;;
                -h|--help)
                    print_usage
                    exit 0
                    ;;
                --channel)
                    ARGS["channel"]="$2"
                    shift 2
                    ;;
                --pmk|--password)
                    ARGS["pmk"]="$2"
                    shift 2
                    ;;
                --mac)
                    ARGS["mac"]="$2"
                    shift 2
                    ;;
                --rate)
                    ARGS["rate"]="$2"
                    shift 2
                    ;;
                *)
                    echo "Error: Unknown argument $1"
                    echo ""
                    print_usage
                    exit 1
                    ;;
            esac
        done
        
        # Validate all required arguments are provided
        local missing_args=()
        for arg in "${REQUIRED_ARGS[@]}"; do
            if [ -z "${ARGS[$arg]}" ]; then
                missing_args+=("--$arg")
            fi
        done
        
        if [ ${#missing_args[@]} -gt 0 ]; then
            echo "Error: Missing required arguments: ${missing_args[*]}"
            echo ""
            print_usage
            exit 1
        fi
    else
        # Parse positional arguments (backward compatibility)
        if [ "$#" -ne ${#REQUIRED_ARGS[@]} ]; then
            echo "Error: Exactly ${#REQUIRED_ARGS[@]} positional arguments required"
            echo ""
            print_usage
            exit 1
        fi
        
        # Assign positional arguments to the ARGS array
        for i in "${!REQUIRED_ARGS[@]}"; do
            local arg_name="${REQUIRED_ARGS[$i]}"
            local arg_index=$((i+1))
            ARGS["$arg_name"]="${!arg_index}"
        done
    fi
    
    # Validate all arguments
    validate_arguments
}

# Check arguments
if [ "$#" -eq 0 ]; then
    echo "Error: No arguments provided"
    echo ""
    print_usage
    exit 1
fi

parse_arguments "$@"

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

echo "Flashing ESP32 ${ARGS[mode]} device..."
echo "  Port: ${ARGS[port]}"
echo "  Target: ${ARGS[target]}"
echo "  Mode: ${ARGS[mode]}"
echo "  Channel: ${ARGS[channel]}"
echo "  PMK: ${ARGS[pmk]}"
echo "  MAC: ${ARGS[mac]}"
echo "  Rate: ${ARGS[rate]}"
echo ""

docker run --rm --privileged \
    -v /dev:/dev \
    "$IMAGE_NAME" "${ARGS[port]}" "${ARGS[target]}" "${ARGS[mode]}" "${ARGS[channel]}" "${ARGS[pmk]}" "${ARGS[mac]}" "${ARGS[rate]}"