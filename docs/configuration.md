# Configuration Reference

This document provides a comprehensive reference for all configuration options in the WiFi CSI Data Collector system.

## Configuration File Structure

The main configuration file is `config/config.yaml`. It uses YAML format with the following main sections:

- `serial`: Serial communication settings
- `data`: Data collection and storage
- `ros2`: ROS2 integration options
- `esp32`: ESP32 device configuration
- `api`: API server settings
- `logging`: Logging configuration
- `advanced`: Advanced features and tuning
- `devices`: Device-specific configurations
- `export`: Data export settings
- `visualization`: Web UI settings
- `security`: Security options

## Serial Communication (`serial`)

Controls communication with ESP32 devices via USB/serial connection.

```yaml
serial:
  port: "/dev/ttyUSB0"          # Serial port device path
  baudrate: 115200              # Communication speed (bps)
  timeout: 5.0                  # Read timeout in seconds
  auto_reconnect: true          # Automatic reconnection on failure
  reconnect_delay: 5.0          # Delay between reconnection attempts
```

### Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | "/dev/ttyUSB0" | Serial device path (Linux/Mac) or COM port (Windows) |
| `baudrate` | integer | 115200 | Serial communication speed. Common values: 115200, 921600 |
| `timeout` | float | 5.0 | Read timeout in seconds |
| `auto_reconnect` | boolean | true | Enable automatic reconnection on connection loss |
| `reconnect_delay` | float | 5.0 | Delay between reconnection attempts |

### Platform-Specific Ports

- **Linux**: `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyACM0`
- **macOS**: `/dev/cu.usbserial-*`, `/dev/cu.SLAB_USBtoUART`
- **Windows**: `COM1`, `COM2`, `COM3`, etc.

## Data Collection (`data`)

Controls how CSI data is collected, processed, and stored.

```yaml
data:
  output_dir: "/app/data"       # Storage directory
  format: "csv"                 # Output format
  max_file_size: 100            # File size limit (MB)
  compression: true             # Enable compression
  buffer_size: 1000             # Sample buffer size
  flush_interval: 30.0          # Disk flush interval (seconds)
```

### Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_dir` | string | "/app/data" | Directory for storing collected data |
| `format` | string | "csv" | Output format: "csv", "json", or "binary" |
| `max_file_size` | integer | 100 | Maximum file size in MB before rotation |
| `compression` | boolean | true | Enable gzip compression for data files |
| `buffer_size` | integer | 1000 | Number of samples to buffer before writing |
| `flush_interval` | float | 30.0 | Interval to flush data to disk |

### Data Formats

#### CSV Format
- Human-readable
- Compatible with Excel, MATLAB, Python pandas
- Includes headers for easy analysis
- Moderate file size

#### JSON Format
- Structured data with metadata
- Easy to parse programmatically
- Larger file size
- Good for complex data structures

#### Binary Format
- Compact storage
- Fastest write speed
- Requires custom parsing
- Smallest file size

## ROS2 Integration (`ros2`)

Configures optional ROS2 communication for robotics applications.

```yaml
ros2:
  enabled: false                # Enable ROS2 bridge
  domain_id: 0                  # ROS2 domain ID
  topic: "/csi_data"            # Publishing topic name
  qos_profile: "default"        # Quality of Service profile
```

### Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | boolean | false | Enable ROS2 bridge functionality |
| `domain_id` | integer | 0 | ROS2 domain ID (0-101) |
| `topic` | string | "/csi_data" | Topic name for publishing CSI data |
| `qos_profile` | string | "default" | QoS profile: "default", "sensor_data", "system_default" |

### QoS Profiles

- **default**: Best effort delivery, keep last 10 messages
- **sensor_data**: Best effort, volatile, keep last 5 messages
- **system_default**: Reliable delivery, transient local

## ESP32 Configuration (`esp32`)

Settings for ESP32 devices and CSI data collection parameters.

```yaml
esp32:
  wifi_ssid: ""                 # Target WiFi network
  wifi_password: ""             # Network password
  csi_rate: 10                  # Sampling rate (Hz)
  channel: 6                    # WiFi channel
  bandwidth: 20                 # Channel bandwidth (MHz)
  filter_mac: ""                # MAC address filter
  enable_debug: false           # Debug output
```

### Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wifi_ssid` | string | "" | WiFi network SSID to monitor |
| `wifi_password` | string | "" | WiFi password (if connection required) |
| `csi_rate` | integer | 10 | CSI sampling rate in Hz (1-100) |
| `channel` | integer | 6 | WiFi channel to monitor (1-13 for 2.4GHz) |
| `bandwidth` | integer | 20 | Channel bandwidth: 20 or 40 MHz |
| `filter_mac` | string | "" | Filter specific MAC address (empty for all) |
| `enable_debug` | boolean | false | Enable debug output from ESP32 |

### Channel Selection

#### 2.4 GHz Channels (1-13)
- **Non-overlapping**: 1, 6, 11
- **Common**: 6 (most widely used)
- **Availability**: Varies by region

#### 5 GHz Channels
- Available on ESP32-S2/S3
- More channels available
- Less congested

### Sampling Rate Guidelines

- **1-5 Hz**: Basic monitoring, energy efficient
- **10-20 Hz**: Standard research applications
- **50-100 Hz**: High-resolution tracking, resource intensive

## API Server (`api`)

Configuration for the REST API and WebSocket server.

```yaml
api:
  host: "0.0.0.0"               # Server bind address
  port: 5000                    # Server port
  cors_origins: ["*"]           # CORS allowed origins
  max_connections: 100          # Maximum concurrent connections
```

### Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | "0.0.0.0" | Server bind address |
| `port` | integer | 5000 | Server port number |
| `cors_origins` | array | ["*"] | CORS allowed origins |
| `max_connections` | integer | 100 | Maximum concurrent connections |

## Logging (`logging`)

Controls system logging behavior.

```yaml
logging:
  level: "INFO"                 # Log level
  file_path: "/app/logs/csi_collector.log"  # Log file location
  max_file_size: 10             # Maximum log file size (MB)
  backup_count: 5               # Number of backup files
```

### Log Levels

- **DEBUG**: Detailed debug information
- **INFO**: General information messages
- **WARNING**: Warning messages
- **ERROR**: Error messages only

## Advanced Settings (`advanced`)

Advanced configuration options for performance tuning and experimental features.

```yaml
advanced:
  enable_real_time_processing: true    # Real-time processing
  processing_threads: 2                # Processing thread count
  serial_read_buffer: 4096            # Serial buffer size
  data_queue_size: 10000              # Internal queue size
  firmware_update_check: true         # Check for updates
  firmware_url: "https://github.com/espressif/esp-csi/releases"
  enable_machine_learning: false      # ML-based processing
  ml_model_path: "/app/models/anomaly_detector.pkl"
  validate_csi_data: true             # Data validation
  discard_invalid_packets: true       # Discard invalid data
  discovery_enabled: true             # Auto device discovery
  discovery_port: 12345               # Discovery UDP port
  enable_alerts: false                # Alert system
  alert_email: ""                     # Alert email address
  alert_webhook: ""                   # Alert webhook URL
```

## Device-Specific Configuration (`devices`)

Individual configurations for multiple ESP32 devices.

```yaml
devices:
  esp32_001:
    name: "Living Room Sensor"
    port: "/dev/ttyUSB0"
    location: "Living Room"
    calibration_offset: 0.0
    enabled: true
  
  esp32_002:
    name: "Bedroom Sensor"
    port: "/dev/ttyUSB1"
    location: "Bedroom"
    calibration_offset: -2.5
    enabled: true
```

### Device Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `name` | string | Human-readable device name |
| `port` | string | Serial port for this device |
| `location` | string | Physical location description |
| `calibration_offset` | float | RSSI calibration offset (dB) |
| `enabled` | boolean | Enable/disable this device |

## Data Export (`export`)

Configuration for data export functionality.

```yaml
export:
  formats: ["csv", "json", "matlab"]  # Available export formats
  include_metadata: true              # Include metadata in exports
  compress_exports: true              # Compress exported files
```

## Visualization (`visualization`)

Web interface and plotting configuration.

```yaml
visualization:
  enable_web_plots: true              # Enable real-time plots
  plot_update_interval: 1.0           # Plot update rate (seconds)
  max_plot_points: 1000               # Maximum points in plots
```

## Security (`security`)

Security settings for production deployments.

```yaml
security:
  enable_authentication: false       # Enable API authentication
  api_key: ""                        # API key for access
  ssl_enabled: false                 # Enable SSL/TLS
  ssl_cert_path: ""                  # SSL certificate path
  ssl_key_path: ""                   # SSL private key path
```

## Environment Variables

Some settings can be overridden with environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| `CONFIG_PATH` | Configuration file path | `/app/config/config.yaml` |
| `DATA_DIR` | Data storage directory | `/app/data` |
| `LOG_LEVEL` | Logging level | `INFO` |
| `ROS_DOMAIN_ID` | ROS2 domain ID | `0` |
| `API_PORT` | API server port | `5000` |

## Configuration Validation

The system validates configuration on startup:

- Serial port accessibility
- Directory permissions
- Parameter ranges
- Required dependencies

### Common Validation Errors

1. **Invalid serial port**: Port doesn't exist or no permission
2. **Invalid sampling rate**: Rate outside 1-100 Hz range
3. **Invalid channel**: Channel outside valid range
4. **Missing directory**: Output directory doesn't exist
5. **ROS2 not available**: ROS2 enabled but not installed

## Configuration Examples

### Basic Research Setup

```yaml
serial:
  port: "/dev/ttyUSB0"
  baudrate: 115200

data:
  format: "csv"
  max_file_size: 50

esp32:
  wifi_ssid: "ResearchNetwork"
  csi_rate: 10
  channel: 6
```

### High-Performance Setup

```yaml
serial:
  port: "/dev/ttyUSB0"
  baudrate: 921600

data:
  format: "binary"
  buffer_size: 5000
  flush_interval: 10.0

esp32:
  csi_rate: 100
  channel: 6
  bandwidth: 20

advanced:
  processing_threads: 4
  serial_read_buffer: 8192
```

### Multi-Device Setup

```yaml
devices:
  sensor_1:
    name: "North Corner"
    port: "/dev/ttyUSB0"
    location: "Building A, Floor 1"
    enabled: true
  
  sensor_2:
    name: "South Corner"
    port: "/dev/ttyUSB1"
    location: "Building A, Floor 1"
    enabled: true
  
  sensor_3:
    name: "Center"
    port: "/dev/ttyUSB2"
    location: "Building A, Floor 1"
    enabled: true
```

### ROS2 Integration Setup

```yaml
ros2:
  enabled: true
  domain_id: 42
  topic: "/sensors/csi_data"
  qos_profile: "sensor_data"

esp32:
  csi_rate: 20
  channel: 6
```

## Best Practices

1. **Start Simple**: Begin with default settings and adjust as needed
2. **Monitor Performance**: Watch CPU and memory usage when tuning
3. **Backup Configurations**: Keep working configurations backed up
4. **Document Changes**: Comment configuration changes
5. **Test Thoroughly**: Test configuration changes in development first