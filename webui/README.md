# CSI Data Logger Web UI

A simple web interface for logging and publishing CSI (Channel State Information) data from ESP32 devices.

## Features

- 🔌 **Persistent Serial Connection** - Connects once and keeps connection alive
- 📊 **Multiple Log Formats** - JSON Lines, CSV, NumPy
- 🤖 **ROS2 Publishing** - Publish CSI data to ROS2 topic in real-time
- 📁 **File Management** - Download and manage log files
- 🔴 **Live Monitoring** - See packets in real-time
- 🐳 **Fully Containerized** - Docker-based deployment
- 🎯 **Queue-Based Architecture** - Multiple consumers (logger + ROS2) from single serial reader

## Architecture

The system uses a **persistent CSI reader** that maintains the serial connection and distributes data to multiple consumers via queues:

```
ESP32 → Serial Port → CSI Reader Manager
                            ↓
                    ┌───────┴────────┐
                    ↓                ↓
                 Logger          ROS2 Publisher
                    ↓                ↓
                Log Files       /csi_data topic
```

### Benefits:
- ✅ Serial port is opened once and shared
- ✅ No port locking issues when stopping/starting logging
- ✅ Can log to file AND publish to ROS2 simultaneously
- ✅ Each consumer operates independently via queues

## Quick Start

### Prerequisites

- Docker and Docker Compose
- ESP32 device connected via USB sending CSI data
- (Optional) ROS2 Humble with `wifi_msgs` package for ROS2 publishing

### Installation

1. **Clone and navigate to webui directory**:
```bash
cd CSIModule/webui
```

2. **Start the service**:
```bash
docker compose up -d
```

3. **Open browser**: http://localhost:5000

4. **Stop the service**:
```bash
docker compose down
```

## Usage

### 1. Connect to Serial Port

1. Click **"Refresh Ports"** to see available ports
2. Select your ESP32 port (e.g., `/dev/ttyACM0`)
3. Click **"Connect"**
4. The persistent reader starts and continuously reads CSI data

3. **Use the interface:**
   - Select your serial port (where ESP32 is connected)
   - Choose log format (JSON, CSV, or NumPy)
   - **(Optional)** Enable ROS2 publishing with the checkbox
   - Click "Start Logging"
   - Monitor live data
   - Stop when done and download the log file

### 3. Enable ROS2 Publishing (Optional)

1. Check the **"Publish CSI data to ROS2"** checkbox
2. Data is published to `/csi_data` topic as `wifi_msgs/msg/CSI`
3. Uncheck to stop publishing

**You can log and publish simultaneously!**

### 4. Monitor Live Data

- The "Live Data" section shows the last 10 packets in real-time
- Status cards show connection state, logging stats, and ROS2 stats

### 5. Disconnect

- Click **"Disconnect"** to stop the reader and close the serial port
- This automatically stops logging and ROS2 publishing

## ROS2 Integration

### Message Type

The system publishes `wifi_msgs/msg/CSI` messages with:

```
std_msgs/Header header
int32 rssi
int32 channel
string bandwidth
int32 noise_floor
int32 rate_index
string mac_address
float64[] amplitude
float64[] phase
int32[] raw_data
```

### Subscribe to CSI Data

```bash
# View messages
ros2 topic echo /csi_data

# Check topic info
ros2 topic info /csi_data

# View message definition
ros2 interface show wifi_msgs/msg/CSI
```

### Python ROS2 Subscriber Example

```python
import rclpy
from rclpy.node import Node
from wifi_msgs.msg import CSI

class CSISubscriber(Node):
    def __init__(self):
        super().__init__('csi_subscriber')
        self.subscription = self.create_subscription(
            CSI,
            'csi_data',
            self.csi_callback,
            10)
    
    def csi_callback(self, msg):
        print(f"RSSI: {msg.rssi}, Channel: {msg.channel}")
        print(f"Amplitude shape: {len(msg.amplitude)}")
        # Process CSI data...

def main():
    rclpy.init()
    subscriber = CSISubscriber()
    rclpy.spin(subscriber)

if __name__ == '__main__':
    main()
```

## Log File Formats

### JSON Lines (.jsonl)
```json
{
  "timestamp": "2025-10-10T12:30:45",
  "packet_num": 1,
  "meta": {"channel": "6", "bandwidth": "20", "rssi": "-45", "mac": "..."},
  "amplitude": [12.3, 15.2, ...],
  "phase": [1.2, -0.5, ...],
  "raw": [...]
}
```

### CSV (.csv)
Standard CSV with headers including timestamp, packet number, metadata (channel, bandwidth, RSSI, etc.), amplitude/phase statistics, and raw CSI data.

### NumPy (.npz)
Compressed NumPy arrays that can be loaded with:
```python
import numpy as np
data = np.load('csi_log_20251010_123045.npz', allow_pickle=True)
timestamps = data['timestamps']
rssi = data['rssi']
channel = data['channel']
amplitude = data['amplitude']  # Array of amplitude arrays
phase = data['phase']          # Array of phase arrays
raw_data = data['raw_data']    # Raw CSI data
```

## ROS2 Integration

The web UI includes optional ROS2 publishing support:

### Setup ROS2 (Optional)

1. **Install ROS2** (Humble or newer recommended):
   - Follow the official ROS2 installation guide for your OS
   - Or use the pre-configured Docker setup (see below)

2. **Install wifi_msgs package**:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/kuederleR/ros2-csi-msgs/refs/heads/main/install.sh | bash
   ```

3. **Enable in the Web UI**:
   - Check the "Publish CSI data to ROS2" checkbox
   - Data will be published to `/csi_data` topic as `wifi_msgs/msg/CSI` messages

### ROS2 Message Format

The `wifi_msgs/msg/CSI` message includes:
- **Header**: Standard ROS timestamp and frame_id
- **Basic Info**: MAC address, RSSI
- **CSI Dimensions**: Number of spatial streams, RX antennas, subcarriers
- **CSI Data**: Complex, amplitude, and phase arrays
- **Metadata**: Channel, bandwidth

### Subscribing to CSI Data

```bash
# List available topics
ros2 topic list

# Echo CSI messages
ros2 topic echo /csi_data

# Get message info
ros2 interface show wifi_msgs/msg/CSI
```

### Docker with ROS2 Support

To run the web UI with ROS2 support in Docker:

1. Build with ROS2 base image (update `Dockerfile`):
   ```dockerfile
   FROM ros:humble-ros-base
   # ... rest of dockerfile
   ```

2. Mount ROS2 workspace:
   ```yaml
   volumes:
     - /opt/ros2-csi-msgs:/opt/ros2-csi-msgs:ro
   environment:
     - ROS_DOMAIN_ID=0
   ```

3. Source ROS2 in container and restart

**Note**: ROS2 publishing is optional and the web UI works perfectly without it. The checkbox will be hidden if ROS2 is not available.

## Log Storage

- Logs are stored in the `./logs` directory
- Files are named with timestamps: `csi_log_YYYYMMDD_HHMMSS.<ext>`
- You can download or delete files from the web UI
- Files persist even after container restarts

## Requirements

- Docker and Docker Compose
- ESP32 device connected via USB/serial sending CSI data
- CSI data output must be compatible with `csi-py` format (ESP-IDF CSI format)

## Configuration

Edit `docker-compose.yml` to change:
- Port mapping (default: 5000)
- Log directory location
- Container privileges

## Development

Run without Docker:
```bash
cd webui
pip install -r requirements.txt
python app.py
```

## Troubleshooting

**No ports showing up?**
- Ensure your ESP32 device is connected
- Check that the container has proper device access (`--privileged` flag)
- Verify USB device permissions on the host

**Data not logging?**
- Confirm your ESP32 is sending CSI data in ESP-IDF format (compatible with csi-py)
- The CSI data should start with "CSI_DATA" in the serial output
- Check the serial baud rate (default: 921600)
- Look at browser console and Docker logs for any errors

**Container can't access devices?**
- Run with `--privileged` flag or add specific device access
- Check host USB permissions: `ls -l /dev/ttyUSB*`

**Blank page at localhost:5000?**
This may be an issue with how MacOS or Windows handles 'host' docker networking. For these operating systems, it is recommended to use a different profile when running the container: 
```bash 
docker compose --profile non-linux up
```

## License

Same as parent project (CSIModule)
