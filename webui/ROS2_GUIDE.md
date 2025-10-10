# ROS2 Publishing Guide for CSI Logger

The CSI Logger Web UI includes built-in ROS2 publishing support using the `wifi_msgs/msg/CSI` message type.

## Current Implementation Status

✅ **Fully Implemented:**
- ROS2 publisher framework with queue-based architecture
- Web UI checkbox control for enabling/disabling ROS2 publishing
- Backend API endpoints (`/api/ros2/start`, `/api/ros2/stop`)
- Status monitoring and packet counting
- Automatic detection of ROS2 availability
- Message conversion from CSIData to `wifi_msgs/msg/CSI`

## How to Use

### 1. Prerequisites

To enable ROS2 publishing, you need:

1. **ROS2 Installation** (Humble or newer):
   ```bash
   # Follow official ROS2 installation guide, or on Ubuntu:
   sudo apt install ros-humble-desktop
   ```

2. **wifi_msgs Package**:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/kuederleR/ros2-csi-msgs/refs/heads/main/install.sh | bash
   source ~/.bashrc
   ```

3. **Rebuild Docker Container with ROS2**:
   The current Dockerfile already uses `ros:humble` as the base image and includes ROS2 support.
   ```bash
   cd webui
   docker compose build
   docker compose up -d
   ```

### 2. Using the Web UI

1. **Open the web interface**: http://localhost:5000

2. **Connect to a serial port**:
   - Select your ESP32 port from the dropdown
   - Click "Connect"

3. **Enable ROS2 Publishing**:
   - Check the ☑️ "Publish CSI data to ROS2" checkbox
   - Data will immediately start publishing to `/csi_data` topic
   - You'll see a green status indicator showing the number of messages published

4. **(Optional) Start Logging**:
   - Select your preferred format (JSON, CSV, or NumPy)
   - Click "Start Logging"
   - Both logging and ROS2 publishing can run simultaneously!

5. **Monitor ROS2 Messages**:
   ```bash
   # In another terminal
   ros2 topic list
   ros2 topic echo /csi_data
   ros2 topic hz /csi_data
   ```

### 3. Message Format

The published `wifi_msgs/msg/CSI` message contains:

```
std_msgs/Header header          # Timestamp and frame ID
string mac                      # MAC address
int32 rssi                      # RSSI in dBm
uint8 nss                       # Number of spatial streams
uint8 nrx                       # Number of receive antennas
uint16 num_subcarriers          # Number of subcarriers
float32[] csi_complex           # Raw complex data [real, imag, real, imag, ...]
float32[] csi_amplitude         # Amplitude values
float32[] csi_phase             # Phase values
uint8 channel                   # WiFi channel number
uint8 bandwidth                 # Bandwidth (20, 40, 80 MHz)
```

### 4. Example ROS2 Subscriber

Create a simple Python subscriber:

```python
#!/usr/bin/env python3
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
        self.get_logger().info(f'Received CSI: RSSI={msg.rssi}, Channel={msg.channel}, Subcarriers={msg.num_subcarriers}')
        # Process amplitude and phase data
        print(f'Amplitude shape: {len(msg.csi_amplitude)}')
        print(f'Phase shape: {len(msg.csi_phase)}')

def main():
    rclpy.init()
    subscriber = CSISubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 csi_subscriber.py
```

## Architecture

The implementation uses a **queue-based architecture**:

1. **CSIReaderManager**: Continuously reads CSI data from the serial port
2. **Queue Distribution**: Each consumer (logger, ROS2 publisher) gets its own queue
3. **ROS2 Publisher Thread**: Runs independently, consuming from its queue and publishing to ROS2
4. **No Blocking**: Serial reading is never blocked by slow consumers

```
ESP32 → Serial Port → CSIReaderManager → Queue → ROS2Publisher → /csi_data topic
                              ↓
                            Queue → CSILogger → Log File
```

## Troubleshooting

### ROS2 checkbox is hidden
- ROS2 is not detected. Check that the container has ROS2 installed
- Look at Docker logs: `docker logs csi-logger | grep ROS2`
- Should see: "ROS2 support enabled"

### "ROS2 is not available" error
- The `wifi_msgs` package is not installed
- Install it with the command from Prerequisites section
- Rebuild the Docker container

### "Not connected to a port" error
- You must connect to a serial port first before enabling ROS2
- Click "Connect" in the Port Management section

### No messages on topic
- Check if publisher is running: `ros2 node list` (should see `/csi_publisher`)
- Check topic: `ros2 topic list` (should see `/csi_data`)
- Verify ESP32 is sending CSI data: look at Live Data section in web UI

### ROS2 domain issues
- Set ROS_DOMAIN_ID if needed:
  ```bash
  export ROS_DOMAIN_ID=0
  # Add to docker-compose.yml:
  environment:
    - ROS_DOMAIN_ID=0
  ```

## Disabling ROS2 Publishing

Simply uncheck the checkbox in the web UI. The publisher will cleanly shut down and release all resources.

## Performance Notes

- ROS2 publishing runs in a separate thread and does not affect serial reading or logging
- Typical performance: 100+ messages/second without issues
- Message publishing is non-blocking
- If ROS2 consumers are slow, messages are queued (default queue size: 100)

## Development Notes

To modify the ROS2 publisher:
- Main code: `webui/app.py` - `ROS2CSIPublisher` class
- Message mapping: `_publish_csi()` method
- UI controls: `webui/templates/index.html` - ROS2 section
- API endpoints: `/api/ros2/start` and `/api/ros2/stop`

## Future Enhancements

Possible improvements:
- [ ] Configurable topic name
- [ ] QoS profile selection
- [ ] Multiple topic publishing
- [ ] TF frame publishing for visualization
- [ ] RViz visualization plugin
