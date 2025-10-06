#!/bin/bash
# Quick start script for CSI Data Collector
# This script provides an easy way to get started with the system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

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

log_header() {
    echo -e "${CYAN}$1${NC}"
}

# Check if Docker is available
check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed or not in PATH"
        echo "Please install Docker and try again."
        echo "Visit: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    if ! docker compose version &> /dev/null; then
        log_error "Docker Compose is not available"
        echo "Please install Docker Compose and try again."
        echo "Visit: https://docs.docker.com/compose/install/"
        exit 1
    fi
    
    # Check if Docker daemon is running
    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running"
        echo "Please start Docker and try again."
        exit 1
    fi
    
    log_success "Docker and Docker Compose are available"
}

# Check system requirements
check_requirements() {
    log_info "Checking system requirements..."
    
    check_docker
    
    # Check for USB devices (ESP32)
    if ls /dev/ttyUSB* &> /dev/null || ls /dev/ttyACM* &> /dev/null; then
        log_success "USB serial devices detected"
    else
        log_warning "No USB serial devices found. Connect your ESP32 device."
    fi
    
    # Check disk space
    available_space=$(df "$PROJECT_DIR" | awk 'NR==2 {print $4}')
    if [ "$available_space" -lt 1000000 ]; then  # Less than 1GB
        log_warning "Low disk space. Consider freeing up space for data collection."
    fi
}

# Setup configuration
setup_config() {
    log_info "Setting up configuration..."
    
    CONFIG_FILE="$PROJECT_DIR/config/config.yaml"
    
    if [ ! -f "$CONFIG_FILE" ]; then
        log_error "Configuration file not found: $CONFIG_FILE"
        exit 1
    fi
    
    # Create data directory
    mkdir -p "$PROJECT_DIR/data"
    
    # Create logs directory
    mkdir -p "$PROJECT_DIR/logs"
    
    log_success "Configuration setup complete"
}

# Start services
start_services() {
    local with_ros2="$1"
    
    log_info "Starting CSI Data Collector services..."
    
    cd "$PROJECT_DIR"
    
    if [ "$with_ros2" = "true" ]; then
        log_info "Starting with ROS2 support..."
        docker compose --profile ros2 up -d
    else
        docker compose up -d
    fi
    
    # Wait a moment for services to start
    sleep 5
    
    # Check if services are running
    if docker compose ps | grep -q "Up"; then
        log_success "Services started successfully"
        
        echo ""
        log_header "=== CSI Data Collector Started ==="
        echo "Web Interface: http://localhost:8080"
        echo "API Endpoint:  http://localhost:8080/api"
        echo ""
        echo "Next steps:"
        echo "1. Open web interface in your browser"
        echo "2. Go to ESP32 Setup tab to configure your device"
        echo "3. Start data collection from the Dashboard"
        echo ""
    else
        log_error "Failed to start services"
        docker compose logs
        exit 1
    fi
}

# Stop services
stop_services() {
    log_info "Stopping CSI Data Collector services..."
    
    cd "$PROJECT_DIR"
    docker compose down
    
    log_success "Services stopped"
}

# Show status
show_status() {
    log_info "Checking service status..."
    
    cd "$PROJECT_DIR"
    docker compose ps
    
    echo ""
    if docker compose ps | grep -q "Up"; then
        log_success "Services are running"
        echo "Web Interface: http://localhost:8080"
    else
        log_warning "Services are not running"
        echo "Use '$0 start' to start the services"
    fi
}

# Show logs
show_logs() {
    local service="$1"
    
    cd "$PROJECT_DIR"
    
    if [ -n "$service" ]; then
        log_info "Showing logs for $service..."
        docker compose logs -f "$service"
    else
        log_info "Showing logs for all services..."
        docker compose logs -f
    fi
}

# ESP32 setup helper
setup_esp32() {
    log_info "Starting ESP32 setup..."
    
    # Check if setup script exists
    SETUP_SCRIPT="$PROJECT_DIR/esp32-setup/setup.sh"
    
    if [ ! -f "$SETUP_SCRIPT" ]; then
        log_error "ESP32 setup script not found: $SETUP_SCRIPT"
        exit 1
    fi
    
    # Make script executable
    chmod +x "$SETUP_SCRIPT"
    
    # Run interactive setup
    "$SETUP_SCRIPT" interactive
}

# Clean up data
clean_data() {
    log_warning "This will delete all collected CSI data!"
    echo -n "Are you sure? (y/N): "
    read -r confirm
    
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        rm -rf "$PROJECT_DIR/data"/*
        rm -rf "$PROJECT_DIR/logs"/*
        log_success "Data cleaned"
    else
        log_info "Data cleanup cancelled"
    fi
}

# Update system
update_system() {
    log_info "Updating CSI Data Collector..."
    
    cd "$PROJECT_DIR"
    
    # Pull latest images
    docker compose pull
    
    # Restart services if they were running
    if docker compose ps | grep -q "Up"; then
        log_info "Restarting services with updated images..."
        docker compose down
        docker compose up -d
        log_success "System updated and restarted"
    else
        log_success "System updated (services were not running)"
    fi
}

# Show usage
show_usage() {
    cat << EOF
CSI Data Collector - Quick Start Script

Usage: $0 <command> [options]

Commands:
    start [--ros2]     Start the CSI data collector services
    stop               Stop all services
    restart [--ros2]   Restart services
    status             Show service status
    logs [service]     Show logs (optionally for specific service)
    setup-esp32        Interactive ESP32 device setup
    clean-data         Remove all collected data (with confirmation)
    update             Update system images and restart if needed
    check              Check system requirements
    help               Show this help message

Examples:
    $0 start                    # Start basic services
    $0 start --ros2             # Start with ROS2 support
    $0 logs csi-collector       # Show collector logs
    $0 setup-esp32              # Configure ESP32 device

Services:
    web-ui              Vue.js web interface
    csi-collector       Python data collection service
    ros2-bridge         ROS2 integration (with --ros2 flag)

Web Interface: http://localhost:8080 (when running)

EOF
}

# Main execution
case "${1:-help}" in
    start)
        check_requirements
        setup_config
        if [[ "$2" == "--ros2" ]]; then
            start_services true
        else
            start_services false
        fi
        ;;
    
    stop)
        stop_services
        ;;
    
    restart)
        stop_services
        sleep 2
        if [[ "$2" == "--ros2" ]]; then
            start_services true
        else
            start_services false
        fi
        ;;
    
    status)
        show_status
        ;;
    
    logs)
        show_logs "$2"
        ;;
    
    setup-esp32)
        setup_esp32
        ;;
    
    clean-data)
        clean_data
        ;;
    
    update)
        update_system
        ;;
    
    check)
        check_requirements
        ;;
    
    help|--help|-h)
        show_usage
        ;;
    
    *)
        log_error "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac