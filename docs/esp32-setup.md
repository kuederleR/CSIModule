# ESP32 Setup Guide

This guide provides detailed instructions for setting up ESP32 devices for WiFi CSI data collection.

## Hardware Requirements

- ESP32 development board (ESP32-DevKitC, ESP32-WROOM-32, or similar)
- USB cable for programming and communication
- Computer with USB port
- WiFi network for CSI monitoring

## Supported ESP32 Boards

- ESP32-DevKitC-32E
- ESP32-WROOM-32
- ESP32-WROVER-KIT
- ESP32-S2-Saola-1
- ESP32-S3-DevKitC-1

## Prerequisites

### Software Dependencies

The setup tools require the following software:

```bash
# Install esptool for firmware flashing
pip install esptool

# Install pyserial for communication
pip install pyserial

# Install PyYAML for configuration
pip install pyyaml
```

### Device Drivers

Install appropriate USB-to-serial drivers:

- **CP210x**: Silicon Labs CP2102/CP2104 (most common)
- **CH340/CH341**: Chinese chips
- **FTDI**: FT232, FT231X chips

## Setup Methods

### Method 1: Interactive Setup Wizard

The easiest way to set up an ESP32 device:

```bash
./esp32-setup/setup.sh interactive
```

This will guide you through:
1. Device detection
2. Firmware selection
3. WiFi configuration
4. CSI parameters
5. Testing and validation

### Method 2: Command Line Setup

For scripted or automated setup:

```bash
# Complete setup in one command
./esp32-setup/setup.sh setup /dev/ttyUSB0 \
    --ssid "YourWiFiNetwork" \
    --password "YourPassword" \
    --firmware stable \
    --rate 10 \
    --channel 6
```

### Method 3: Manual Step-by-Step

For advanced users or troubleshooting:

```bash
# 1. Scan for devices
./esp32-setup/setup.sh scan

# 2. Flash firmware
./esp32-setup/setup.sh flash /dev/ttyUSB0 --firmware stable

# 3. Configure device
./esp32-setup/setup.sh configure /dev/ttyUSB0 \
    --ssid "YourWiFiNetwork" \
    --password "YourPassword" \
    --rate 10 \
    --channel 6

# 4. Test connection
./esp32-setup/setup.sh test /dev/ttyUSB0
```

## Firmware Options

### Stable Release (Recommended)

- Pre-compiled binary
- Tested and stable
- Suitable for most applications

### Latest Development

- Latest features
- May include experimental functionality
- Requires building from source

### Custom Firmware

- Upload your own firmware binary
- For specialized applications
- Requires ESP-IDF knowledge

## Configuration Parameters

### WiFi Settings

- **SSID**: Network name to monitor
- **Password**: Network password (if required for connection)
- **Channel**: WiFi channel (1-13)
- **Bandwidth**: 20MHz or 40MHz

### CSI Settings

- **Sampling Rate**: 1-100 Hz (higher rates require more processing)
- **Filter MAC**: Specific device to monitor (optional)
- **Debug Mode**: Enable detailed logging

### Serial Communication

- **Baud Rate**: 115200 (default) or 921600 (high speed)
- **Port**: USB device path (e.g., /dev/ttyUSB0)

## Troubleshooting

### Device Not Detected

1. **Check USB connection**:
   ```bash
   # List USB devices
   lsusb
   
   # Check serial ports
   ls -la /dev/ttyUSB*
   ls -la /dev/ttyACM*
   ```

2. **Install drivers**:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install linux-headers-$(uname -r)
   
   # For CH340 chips
   sudo apt-get install ch341-uart-dkms
   ```

3. **Check permissions**:
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

### Flash Operation Failed

1. **Put ESP32 in download mode**:
   - Hold BOOT button
   - Press and release RESET button
   - Release BOOT button
   - Try flashing again

2. **Check for interference**:
   - Close other serial applications
   - Disconnect other devices
   - Try different USB port/cable

3. **Reduce baud rate**:
   ```bash
   ./esp32-setup/setup.sh flash /dev/ttyUSB0 --baud 115200
   ```

### Configuration Not Applied

1. **Verify device response**:
   ```bash
   # Manual communication test
   screen /dev/ttyUSB0 115200
   # Type: STATUS
   # Should receive configuration info
   ```

2. **Check WiFi connectivity**:
   - Ensure SSID is correct
   - Verify password
   - Check WiFi range

3. **Reset device**:
   ```bash
   # Send reset command
   ./esp32-setup/setup.sh configure /dev/ttyUSB0 --reset
   ```

### No CSI Data Received

1. **Check WiFi activity**:
   - Ensure target network has activity
   - Try different channel
   - Check for interference

2. **Verify sampling rate**:
   - Start with low rate (1-5 Hz)
   - Gradually increase if needed

3. **Monitor device logs**:
   ```bash
   # Enable debug mode
   ./esp32-setup/setup.sh configure /dev/ttyUSB0 --debug true
   ```

## Advanced Configuration

### Multiple Device Setup

For research setups with multiple ESP32 devices:

```bash
# Setup device 1
./esp32-setup/setup.sh setup /dev/ttyUSB0 --ssid WiFi --channel 1

# Setup device 2
./esp32-setup/setup.sh setup /dev/ttyUSB1 --ssid WiFi --channel 6

# Setup device 3
./esp32-setup/setup.sh setup /dev/ttyUSB2 --ssid WiFi --channel 11
```

### Custom Firmware Building

For advanced users wanting to build custom firmware:

```bash
# Clone ESP-CSI repository
git clone https://github.com/espressif/esp-csi.git
cd esp-csi

# Setup ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Configure project
idf.py menuconfig

# Build firmware
idf.py build

# Flash custom firmware
./esp32-setup/setup.sh flash /dev/ttyUSB0 --firmware-file build/esp-csi.bin
```

### Automated Deployment

For large-scale deployments:

```bash
#!/bin/bash
# bulk_setup.sh - Setup multiple devices

DEVICES=("/dev/ttyUSB0" "/dev/ttyUSB1" "/dev/ttyUSB2")
SSID="ResearchNetwork"
PASSWORD="SecurePassword"

for device in "${DEVICES[@]}"; do
    echo "Setting up $device..."
    ./esp32-setup/setup.sh setup "$device" \
        --ssid "$SSID" \
        --password "$PASSWORD" \
        --rate 10 \
        --channel 6
done
```

## Best Practices

1. **Device Labeling**: Label devices with their configuration
2. **Cable Management**: Use high-quality USB cables
3. **Power Supply**: Ensure stable power supply
4. **Environmental Considerations**: Protect from moisture and dust
5. **Backup Configuration**: Keep configuration files for easy re-deployment

## Performance Considerations

### Sampling Rate Guidelines

- **1-5 Hz**: Basic monitoring, long-term studies
- **10-20 Hz**: Standard research applications
- **50-100 Hz**: High-resolution movement tracking

### Channel Selection

- **2.4 GHz Channels**: 1, 6, 11 (non-overlapping)
- **5 GHz Support**: Available on ESP32-S2/S3
- **Channel Scanning**: Rotate through multiple channels

## Security Considerations

1. **Data Privacy**: Ensure compliance with local regulations
2. **Network Access**: Use dedicated research networks when possible
3. **Data Encryption**: Enable encryption for sensitive applications
4. **Access Control**: Restrict physical access to devices

## Maintenance

### Regular Tasks

- Check device connectivity weekly
- Update firmware quarterly
- Backup collected data regularly
- Monitor system logs for errors

### Diagnostic Commands

```bash
# Check device status
./esp32-setup/setup.sh test /dev/ttyUSB0

# View device information
./esp32-setup/setup.sh info /dev/ttyUSB0

# Reset device configuration
./esp32-setup/setup.sh reset /dev/ttyUSB0
```