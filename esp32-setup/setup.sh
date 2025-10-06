#!/bin/bash
# ESP32 Setup Automation Script
# Provides easy command-line interface for ESP32 setup operations

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/setup_esp32.py"
CONFIG_FILE="$SCRIPT_DIR/../config/config.yaml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Python script exists
if [ ! -f "$PYTHON_SCRIPT" ]; then
    log_error "Python setup script not found: $PYTHON_SCRIPT"
    exit 1
fi

# Function to show usage
show_usage() {
    cat << EOF
ESP32 Setup Automation Tool

Usage: $0 <command> [options]

Commands:
    scan                    Scan for available ESP32 devices
    flash <port>           Flash ESP-CSI firmware to device
    configure <port>       Configure device with WiFi settings
    test <port>            Test device connection and data flow
    setup <port>           Complete setup (flash + configure + test)
    interactive            Interactive setup wizard
    list-configs           List available configuration templates

Options:
    --ssid <ssid>          WiFi SSID for device configuration
    --password <pass>      WiFi password
    --firmware <type>      Firmware type: stable (default) or latest
    --rate <hz>            CSI sampling rate in Hz (default: 10)
    --channel <ch>         WiFi channel 1-13 (default: 6)
    --config <file>        Custom configuration file
    --help                 Show this help message

Examples:
    $0 scan                                    # Scan for devices
    $0 flash /dev/ttyUSB0                     # Flash firmware
    $0 setup /dev/ttyUSB0 --ssid MyWiFi       # Complete setup
    $0 interactive                            # Interactive wizard

EOF
}

# Function to scan for devices
scan_devices() {
    log_info "Scanning for ESP32 devices..."
    python3 "$PYTHON_SCRIPT" scan --config "$CONFIG_FILE"
}

# Function to flash firmware
flash_firmware() {
    local port="$1"
    local firmware="${2:-stable}"
    
    if [ -z "$port" ]; then
        log_error "Port not specified for flash operation"
        return 1
    fi
    
    log_info "Flashing $firmware firmware to $port..."
    if python3 "$PYTHON_SCRIPT" flash --port "$port" --firmware "$firmware" --config "$CONFIG_FILE"; then
        log_success "Firmware flashed successfully"
        return 0
    else
        log_error "Failed to flash firmware"
        return 1
    fi
}

# Function to configure device
configure_device() {
    local port="$1"
    local ssid="$2"
    local password="$3"
    local rate="${4:-10}"
    local channel="${5:-6}"
    
    if [ -z "$port" ] || [ -z "$ssid" ]; then
        log_error "Port and SSID are required for configuration"
        return 1
    fi
    
    log_info "Configuring device on $port..."
    if python3 "$PYTHON_SCRIPT" configure --port "$port" --ssid "$ssid" --password "$password" --rate "$rate" --channel "$channel" --config "$CONFIG_FILE"; then
        log_success "Device configured successfully"
        return 0
    else
        log_error "Failed to configure device"
        return 1
    fi
}

# Function to test device
test_device() {
    local port="$1"
    local duration="${2:-5.0}"
    
    if [ -z "$port" ]; then
        log_error "Port not specified for test"
        return 1
    fi
    
    log_info "Testing device on $port..."
    if python3 "$PYTHON_SCRIPT" test --port "$port" --duration "$duration" --config "$CONFIG_FILE"; then
        log_success "Device test completed successfully"
        return 0
    else
        log_error "Device test failed"
        return 1
    fi
}

# Function for complete setup
complete_setup() {
    local port="$1"
    local ssid="$2"
    local password="$3"
    local firmware="${4:-stable}"
    
    if [ -z "$port" ] || [ -z "$ssid" ]; then
        log_error "Port and SSID are required for setup"
        return 1
    fi
    
    log_info "Starting complete setup for device on $port..."
    if python3 "$PYTHON_SCRIPT" setup --port "$port" --ssid "$ssid" --password "$password" --firmware "$firmware" --config "$CONFIG_FILE"; then
        log_success "Complete setup finished successfully"
        return 0
    else
        log_error "Setup failed"
        return 1
    fi
}

# Interactive setup wizard
interactive_setup() {
    log_info "Starting interactive ESP32 setup wizard..."
    echo
    
    # Step 1: Scan for devices
    log_info "Step 1: Scanning for available devices..."
    python3 "$PYTHON_SCRIPT" scan --config "$CONFIG_FILE"
    echo
    
    # Step 2: Select device
    echo -n "Enter the serial port for your ESP32 device (e.g., /dev/ttyUSB0): "
    read -r PORT
    
    if [ -z "$PORT" ]; then
        log_error "No port specified"
        return 1
    fi
    
    # Step 3: WiFi configuration
    echo -n "Enter WiFi SSID: "
    read -r SSID
    
    if [ -z "$SSID" ]; then
        log_error "No SSID specified"
        return 1
    fi
    
    echo -n "Enter WiFi password (press Enter for open network): "
    read -r -s PASSWORD
    echo
    
    # Step 4: CSI settings
    echo -n "Enter CSI sampling rate in Hz (default: 10): "
    read -r RATE
    RATE="${RATE:-10}"
    
    echo -n "Enter WiFi channel 1-13 (default: 6): "
    read -r CHANNEL
    CHANNEL="${CHANNEL:-6}"
    
    # Step 5: Firmware selection
    echo -n "Select firmware type - stable/latest (default: stable): "
    read -r FIRMWARE
    FIRMWARE="${FIRMWARE:-stable}"
    
    # Confirmation
    echo
    log_info "Setup Configuration:"
    echo "  Port: $PORT"
    echo "  SSID: $SSID"
    echo "  Rate: $RATE Hz"
    echo "  Channel: $CHANNEL"
    echo "  Firmware: $FIRMWARE"
    echo
    
    echo -n "Proceed with setup? (y/N): "
    read -r CONFIRM
    
    if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
        log_warning "Setup cancelled by user"
        return 0
    fi
    
    # Execute setup
    complete_setup "$PORT" "$SSID" "$PASSWORD" "$FIRMWARE"
}

# Function to list configuration templates
list_configs() {
    log_info "Available configuration templates:"
    echo
    echo "Default Configuration:"
    echo "  Sampling Rate: 10 Hz"
    echo "  Channel: 6"
    echo "  Bandwidth: 20 MHz"
    echo "  Format: CSV"
    echo
    echo "High Rate Configuration:"
    echo "  Sampling Rate: 50 Hz"
    echo "  Channel: 6"
    echo "  Bandwidth: 20 MHz"
    echo "  Format: Binary"
    echo
    echo "Research Configuration:"
    echo "  Sampling Rate: 100 Hz"
    echo "  Channel: All (scanning)"
    echo "  Bandwidth: 40 MHz"
    echo "  Format: JSON"
    echo
    log_info "Use --config <file> to specify custom configuration"
}

# Parse command line arguments
COMMAND=""
PORT=""
SSID=""
PASSWORD=""
FIRMWARE="stable"
RATE="10"
CHANNEL="6"
CUSTOM_CONFIG=""

while [[ $# -gt 0 ]]; do
    case $1 in
        scan|flash|configure|test|setup|interactive|list-configs)
            COMMAND="$1"
            shift
            ;;
        --ssid)
            SSID="$2"
            shift 2
            ;;
        --password)
            PASSWORD="$2"
            shift 2
            ;;
        --firmware)
            FIRMWARE="$2"
            shift 2
            ;;
        --rate)
            RATE="$2"
            shift 2
            ;;
        --channel)
            CHANNEL="$2"
            shift 2
            ;;
        --config)
            CUSTOM_CONFIG="$2"
            CONFIG_FILE="$2"
            shift 2
            ;;
        --help|-h)
            show_usage
            exit 0
            ;;
        *)
            if [ -z "$PORT" ] && [[ "$1" =~ ^/dev/ ]]; then
                PORT="$1"
            else
                log_error "Unknown option: $1"
                show_usage
                exit 1
            fi
            shift
            ;;
    esac
done

# Main execution
case "$COMMAND" in
    scan)
        scan_devices
        ;;
    flash)
        flash_firmware "$PORT" "$FIRMWARE"
        ;;
    configure)
        configure_device "$PORT" "$SSID" "$PASSWORD" "$RATE" "$CHANNEL"
        ;;
    test)
        test_device "$PORT"
        ;;
    setup)
        complete_setup "$PORT" "$SSID" "$PASSWORD" "$FIRMWARE"
        ;;
    interactive)
        interactive_setup
        ;;
    list-configs)
        list_configs
        ;;
    "")
        log_error "No command specified"
        show_usage
        exit 1
        ;;
    *)
        log_error "Unknown command: $COMMAND"
        show_usage
        exit 1
        ;;
esac