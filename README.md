# ESP32 CSI Flash Tool

A streamlined Docker-based solution for configuring and flashing ESP32 modules as CSI (Channel State Information) receivers or senders. This tool simplifies the process of building and deploying ESP32 CSI applications with a simple command-line interface.

## Features

- 🐳 **Docker-based**: Completely containerized with ESP-IDF toolchain
- 🚀 **Simple CLI**: Single command to configure and flash ESP32 modules
- ⚙️ **Flexible Configuration**: Command-line options or YAML configuration files
- 🔌 **Auto-detection**: Automatic serial port detection
- 📊 **Rich Output**: Beautiful terminal output with progress indicators
- 🎯 **Pre-configured**: Ready-to-use templates for common use cases

## Quick Start

### 1. Build the Docker Image

```bash
./flash_csi_docker.sh build
```

### 2. Flash a CSI Receiver

```bash
# Auto-detect serial port and use defaults
./flash_csi_docker.sh receiver

# Specify port and channel
./flash_csi_docker.sh receiver --port /dev/ttyUSB0 --channel 11
```

### 3. Flash a CSI Sender

```bash
# Auto-detect serial port and use defaults  
./flash_csi_docker.sh sender

# Specify port, channel, and WiFi credentials
./flash_csi_docker.sh sender --port /dev/ttyUSB1 --channel 11 --ssid "MyWiFi" --password "MyPassword"
```

## Installation

### Prerequisites

- Docker and Docker Compose
- Linux host with USB access to ESP32 devices
- ESP32 development boards (ESP32, ESP32-S3, ESP32-C6, etc.)

### Setup

1. Clone this repository:
```bash
git clone <repository-url>
cd CSIModule
```

2. Build the Docker image:
```bash
./flash_csi_docker.sh build
```

3. Verify installation:
```bash
./flash_csi_docker.sh list-ports
```

## Usage

### Basic Commands

```bash
# Build Docker image
./flash_csi_docker.sh build

# List available serial ports
./flash_csi_docker.sh list-ports

# Interactive shell for advanced usage
./flash_csi_docker.sh shell

# Clean up Docker containers and images
./flash_csi_docker.sh clean
```

### Flashing ESP32 as CSI Receiver

```bash
# Basic receiver with auto-detection
./flash_csi_docker.sh receiver

# Receiver with specific configuration
./flash_csi_docker.sh receiver \
  --port /dev/ttyUSB0 \
  --channel 6 \
  --target esp32c6 \
  --baudrate 921600

# Skip monitor after flashing
./flash_csi_docker.sh receiver --no-monitor
```

### Flashing ESP32 as CSI Sender

```bash
# Basic sender with auto-detection
./flash_csi_docker.sh sender

# Sender with specific configuration
./flash_csi_docker.sh sender \
  --port /dev/ttyUSB1 \
  --channel 6 \
  --frequency 50 \
  --ssid "MyCSI_AP" \
  --password "MyPassword"

# High frequency sender
./flash_csi_docker.sh sender --frequency 100
```

### Using Configuration Files

1. Generate a configuration template:
```bash
./flash_csi_docker.sh generate-config receiver_config.yaml --mode receiver
./flash_csi_docker.sh generate-config sender_config.yaml --mode sender
```

2. Edit the configuration file:
```yaml
target: esp32c6
channel: 11
bandwidth: HT20
csi_enabled: true
uart_baudrate: 921600
buffer_num: 256
ampdu_tx_enabled: false
```

3. Use the configuration file:
```bash
./flash_csi_docker.sh receiver --config-file receiver_config.yaml
```

## Configuration Options

### Common Options

| Option | Description | Default |
|--------|-------------|---------|
| `--port` | Serial port (e.g., /dev/ttyUSB0) | Auto-detect |
| `--channel` | WiFi channel (1-14) | 6 |
| `--target` | ESP32 target chip | esp32c6 |
| `--baudrate` | UART baudrate | 921600 |
| `--no-monitor` | Skip monitor after flashing | false |
| `--config-file` | YAML configuration file | - |

### Receiver-Specific Options

| Option | Description | Default |
|--------|-------------|---------|
| `--buffer-num` | WiFi RX buffer number | 128 |

### Sender-Specific Options

| Option | Description | Default |
|--------|-------------|---------|
| `--frequency` | Send frequency in Hz | 20 |
| `--ssid` | WiFi SSID | MyCSI_AP |
| `--password` | WiFi password | MyCSI_Password |

### Supported ESP32 Targets

- `esp32` - Original ESP32
- `esp32s3` - ESP32-S3
- `esp32c6` - ESP32-C6 (default)
- `esp32c5` - ESP32-C5

## Advanced Usage

### Interactive Shell

For advanced debugging and development:

```bash
./flash_csi_docker.sh shell
```

Inside the container, you can:
- Access the Python CLI directly: `python /workspace/docker/csi_flash_tool.py --help`
- Examine project files: `ls /workspace/esp/`
- Run ESP-IDF commands manually: `idf.py --help`

### Manual Docker Commands

If you prefer using Docker directly:

```bash
# Build image
docker build -t esp32-csi-flash .

# Run receiver flashing
docker run -it --rm --privileged \
  -v /dev:/dev \
  esp32-csi-flash receiver --port /dev/ttyUSB0

# Run with volume mounts for configs
docker run -it --rm --privileged \
  -v /dev:/dev \
  -v $(pwd)/configs:/workspace/configs \
  esp32-csi-flash receiver --config-file /workspace/configs/my_config.yaml
```

### Configuration Templates

The tool includes predefined configurations in `docker/config_templates.yaml`:

- `basic_pair` - Basic CSI receiver/sender pair
- `high_frequency` - High-frequency CSI pair
- `multi_channel` - Multi-channel setup
- `esp32_basic` - Original ESP32 configurations
- `esp32s3_basic` - ESP32-S3 configurations

## Troubleshooting

### Common Issues

**Permission denied for /dev/ttyUSB***
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER
# Log out and back in, or use:
newgrp dialout
```

**Docker permission denied**
```bash
# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

**No serial ports found**
- Check if ESP32 is connected: `lsusb`
- Check if device appears: `ls /dev/tty*`
- Try different USB cable or port

**Build failures**
- Ensure ESP32 target is correct for your hardware
- Check available memory on Docker host
- Try cleaning and rebuilding: `./flash_csi_docker.sh clean && ./flash_csi_docker.sh build`

### Debugging

Enable verbose output:
```bash
# Add debug information
./flash_csi_docker.sh shell
# Inside container:
export ESP_LOG_LEVEL=DEBUG
python /workspace/docker/csi_flash_tool.py receiver --port /dev/ttyUSB0
```

View container logs:
```bash
docker logs esp32-csi-flash
```

### Port Detection Issues

List all USB devices:
```bash
lsusb
```

List all TTY devices:
```bash
ls -la /dev/tty*
```

Check device permissions:
```bash
ls -la /dev/ttyUSB*
```

## Development

### Project Structure

```
CSIModule/
├── esp/                          # ESP32 source code
│   ├── csi_recv/                 # CSI receiver project
│   ├── csi_send/                 # CSI sender project
│   └── flash_csi.sh             # Original flash script
├── docker/                      # Docker-related files
│   ├── csi_flash_tool.py        # Main Python CLI tool
│   └── config_templates.yaml    # Configuration templates
├── Dockerfile                   # Docker image definition
├── docker-compose.yml          # Docker Compose configuration
├── flash_csi_docker.sh         # Main helper script
└── README.md                   # This file
```

### Extending the Tool

To add new features:

1. **New ESP32 targets**: Update `DEFAULT_CONFIGS` in `csi_flash_tool.py`
2. **New configuration options**: Add CLI options and update configuration generation
3. **New build targets**: Modify the `build_and_flash` method

### Contributing

1. Fork the repository
2. Create a feature branch
3. Test your changes with multiple ESP32 targets
4. Submit a pull request

## License

This project is licensed under the terms specified in the LICENSE file.

## Acknowledgments

- Espressif Systems for ESP-IDF framework
- ESP32 CSI community for research and development
- Docker community for containerization tools
