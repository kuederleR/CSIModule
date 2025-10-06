# WiFi CSI Data Collector

A comprehensive system for collecting, processing, and analyzing WiFi Channel State Information (CSI) data using ESP32 devices. This project provides a complete Docker-based solution with web interface, real-time data collection, optional ROS2 integration, and automated ESP32 setup tools.

## 🌟 Features

- **🔧 Automated ESP32 Setup**: Automatic firmware flashing and device configuration
- **🌐 Web Interface**: Real-time data visualization and system monitoring
- **📊 Multiple Data Formats**: CSV, JSON, and binary output support
- **🤖 ROS2 Integration**: Optional ROS2 Humble communication for robotics research
- **📱 Real-time Monitoring**: WebSocket-based live data streaming
- **🐳 Docker-based**: Easy deployment with Docker Compose
- **⚙️ Flexible Configuration**: Comprehensive YAML-based configuration system
- **📈 Data Analysis**: Built-in tools for CSI data analysis and visualization

## 🚀 Quick Start

### Prerequisites

- Docker and Docker Compose
- ESP32 development board with ESP-CSI firmware support
- USB connection to ESP32 device

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/kuederleR/CSIModule.git
   cd CSIModule
   ```

2. **Configure the system**
   ```bash
   # Edit the configuration file
   nano config/config.yaml
   ```

3. **Start the services**
   ```bash
   # Standard setup
   docker-compose up -d
   
   # With ROS2 support
   docker-compose --profile ros2 up -d
   ```

4. **Access the web interface**
   ```
   http://localhost:8080
   ```

## 📋 Setup Guide

### ESP32 Device Setup

The system includes automated tools for ESP32 setup:

```bash
# Interactive setup wizard
./esp32-setup/setup.sh interactive

# Quick setup
./esp32-setup/setup.sh setup /dev/ttyUSB0 --ssid YourWiFiNetwork

# Manual steps
./esp32-setup/setup.sh scan                    # Find devices
./esp32-setup/setup.sh flash /dev/ttyUSB0      # Flash firmware
./esp32-setup/setup.sh configure /dev/ttyUSB0 --ssid YourWiFi  # Configure
./esp32-setup/setup.sh test /dev/ttyUSB0       # Test connection
```

### Configuration

The main configuration file is `config/config.yaml`. Key settings include:

```yaml
# Serial Communication
serial:
  port: "/dev/ttyUSB0"
  baudrate: 115200

# Data Collection
data:
  output_dir: "/app/data"
  format: "csv"
  max_file_size: 100

# ROS2 Integration (optional)
ros2:
  enabled: false
  domain_id: 0
  topic: "/csi_data"

# ESP32 Settings
esp32:
  wifi_ssid: "YourWiFiNetwork"
  csi_rate: 10
  channel: 6
```

## 🏗️ Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Web UI        │    │  CSI Collector  │    │   ESP32 Setup   │
│  (Vue.js/Nginx) │    │   (Python)      │    │    Tools        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                         ┌───────▼───────┐
                         │    Docker     │
                         │   Network     │
                         └───────────────┘
                                 │
                         ┌───────▼───────┐
                         │   ESP32       │
                         │   Device      │
                         └───────────────┘
```

### Components

- **Web UI**: Vue.js-based interface for monitoring and control
- **CSI Collector**: Python service for data collection and processing
- **ROS2 Bridge**: Optional ROS2 integration for robotics applications
- **ESP32 Setup**: Automated tools for device configuration
- **Configuration System**: YAML-based configuration management

## 📁 Project Structure

```
CSIModule/
├── web-ui/                 # Vue.js web interface
│   ├── src/
│   │   ├── components/     # Vue components
│   │   └── main.js        # Application entry point
│   ├── Dockerfile         # Web UI container
│   └── package.json       # Dependencies
├── csi-collector/          # Python data collector
│   ├── src/
│   │   ├── collector.py   # Main collection logic
│   │   ├── api_server.py  # REST API and WebSocket
│   │   ├── ros2_bridge.py # ROS2 integration
│   │   └── config_manager.py # Configuration handling
│   ├── Dockerfile         # Collector container
│   └── requirements.txt   # Python dependencies
├── esp32-setup/           # ESP32 automation tools
│   ├── setup_esp32.py     # Python setup script
│   └── setup.sh          # Bash wrapper script
├── config/
│   └── config.yaml        # Main configuration
├── data/                  # Data storage directory
├── docs/                  # Documentation
└── docker-compose.yml     # Container orchestration
```

## 🔧 Usage Examples

### Basic Data Collection

1. **Start the system**:
   ```bash
   docker-compose up -d
   ```

2. **Configure ESP32** (via web interface or CLI):
   ```bash
   ./esp32-setup/setup.sh setup /dev/ttyUSB0 --ssid MyWiFi
   ```

3. **Monitor data collection** at http://localhost:8080

### ROS2 Integration

1. **Enable ROS2 in configuration**:
   ```yaml
   ros2:
     enabled: true
     domain_id: 0
     topic: "/csi_data"
   ```

2. **Start with ROS2 profile**:
   ```bash
   docker-compose --profile ros2 up -d
   ```

3. **Subscribe to CSI data**:
   ```bash
   ros2 topic echo /csi_data
   ```

### Data Analysis

```python
import pandas as pd
import json

# Load CSI data
df = pd.read_csv('data/csi_data_20251006_143000.csv')

# Parse amplitude data
df['amplitude_data'] = df['amplitude'].apply(json.loads)

# Basic analysis
print(f"Total packets: {len(df)}")
print(f"Average RSSI: {df['rssi'].mean():.2f} dBm")
print(f"Data collection period: {df['timestamp'].max() - df['timestamp'].min():.2f} seconds")
```

## 🌐 API Reference

### REST Endpoints

- `GET /health` - Health check
- `GET /status` - System status
- `POST /start` - Start data collection
- `POST /stop` - Stop data collection
- `GET /config` - Get configuration
- `POST /config` - Update configuration
- `GET /data/files` - List data files
- `GET /data/file/{filename}` - Get data file content
- `GET /esp32/scan` - Scan for ESP32 devices
- `POST /esp32/flash` - Flash firmware
- `POST /esp32/configure` - Configure device

### WebSocket Events

- `csi_data` - Real-time CSI data
- `status_update` - System status updates
- `device_status` - Device connection status

## 🔬 Research Applications

This system is designed for various research applications:

- **Indoor Localization**: Track device movement using CSI patterns
- **Activity Recognition**: Detect human activities through WiFi signal changes
- **Environmental Monitoring**: Monitor environmental changes affecting RF propagation
- **Security Research**: Detect intrusions and anomalies in wireless networks
- **IoT Applications**: Integrate with IoT systems for enhanced sensing capabilities

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 📚 Documentation

- [ESP32 Setup Guide](docs/esp32-setup.md)
- [Configuration Reference](docs/configuration.md)
- [API Documentation](docs/api-reference.md)
- [Data Format Specification](docs/data-format.md)
- [ROS2 Integration Guide](docs/ros2-integration.md)

## 🐛 Troubleshooting

### Common Issues

1. **ESP32 not detected**:
   - Check USB connection
   - Verify device drivers
   - Try different USB port

2. **Permission denied on serial port**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **Docker permission issues**:
   ```bash
   sudo usermod -a -G docker $USER
   # Log out and back in
   ```

4. **ROS2 integration not working**:
   - Ensure ROS2 Humble is installed
   - Check ROS_DOMAIN_ID environment variable
   - Verify network connectivity

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [ESP-CSI](https://github.com/espressif/esp-csi) - Espressif's CSI implementation
- [ROS2](https://ros.org/) - Robot Operating System 2
- [Vue.js](https://vuejs.org/) - Progressive JavaScript framework
- [Flask](https://flask.palletsprojects.com/) - Python web framework

## 📧 Support

For questions and support:

- Create an issue on GitHub
- Check the documentation in the `docs/` directory
- Review the configuration examples

---

**Note**: This project is designed for research and educational purposes. Ensure compliance with local regulations regarding WiFi monitoring and data collection.
A repository for a dedicated WiFi CSI module designed for research purposes.
