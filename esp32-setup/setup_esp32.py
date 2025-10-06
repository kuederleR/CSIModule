#!/usr/bin/env python3
"""
ESP32 Setup and Configuration Tools
Automates ESP-CSI firmware installation and device configuration
"""

import argparse
import logging
import os
import subprocess
import sys
import time
import urllib.request
import zipfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports
import yaml

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class ESP32SetupManager:
    """Manages ESP32 firmware installation and configuration"""
    
    FIRMWARE_URLS = {
        'stable': 'https://github.com/espressif/esp-csi/releases/latest/download/esp-csi-firmware.bin',
        'latest': 'https://github.com/espressif/esp-csi/archive/refs/heads/master.zip'
    }
    
    ESP_CSI_CONFIG_TEMPLATE = """
{
    "wifi_ssid": "{ssid}",
    "wifi_password": "{password}",
    "device_mode": "{device_mode}",
    "csi_rate": {csi_rate},
    "channel": {channel},
    "bandwidth": {bandwidth},
    "tx_power": {tx_power},
    "beacon_interval": {beacon_interval},
    "filter_mac": "{filter_mac}",
    "enable_debug": {debug}
}
"""
    
    def __init__(self, config_path: str = None):
        self.config_path = config_path
        self.config = self._load_config()
        self.firmware_dir = Path('./firmware')  # Use relative path
        self.firmware_dir.mkdir(exist_ok=True)
    
    def _load_config(self) -> Dict:
        """Load configuration from YAML file"""
        if not self.config_path or not os.path.exists(self.config_path):
            return {}
        
        try:
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            logger.warning(f"Failed to load config: {e}")
            return {}
    
    def scan_devices(self) -> List[Dict[str, str]]:
        """Scan for available ESP32 devices"""
        logger.info("Scanning for ESP32 devices...")
        
        devices = []
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            device_info = {
                'device': port.device,
                'description': port.description,
                'manufacturer': getattr(port, 'manufacturer', 'Unknown'),
                'serial_number': getattr(port, 'serial_number', 'Unknown'),
                'vid': getattr(port, 'vid', None),
                'pid': getattr(port, 'pid', None)
            }
            
            # Check if it's likely an ESP32
            is_esp32 = False
            if port.description:
                esp_keywords = ['esp32', 'silicon labs', 'cp210', 'ch340', 'ftdi']
                if any(keyword in port.description.lower() for keyword in esp_keywords):
                    is_esp32 = True
            
            if port.vid:
                # Common VID/PID combinations for ESP32 development boards
                esp32_ids = [
                    (0x10C4, 0xEA60),  # Silicon Labs CP210x
                    (0x1A86, 0x7523),  # CH340
                    (0x0403, 0x6001),  # FTDI
                    (0x303A, 0x1001),  # Espressif ESP32-S2
                ]
                if (port.vid, port.pid) in esp32_ids:
                    is_esp32 = True
            
            device_info['is_esp32'] = is_esp32
            devices.append(device_info)
        
        esp32_devices = [d for d in devices if d['is_esp32']]
        logger.info(f"Found {len(esp32_devices)} potential ESP32 devices")
        
        return devices
    
    def download_firmware(self, firmware_type: str = 'stable') -> Optional[str]:
        """Download ESP-CSI firmware"""
        if firmware_type not in self.FIRMWARE_URLS:
            logger.error(f"Unknown firmware type: {firmware_type}")
            return None
        
        url = self.FIRMWARE_URLS[firmware_type]
        filename = f"esp-csi-{firmware_type}.bin"
        firmware_path = self.firmware_dir / filename
        
        logger.info(f"Downloading firmware from {url}")
        
        try:
            urllib.request.urlretrieve(url, firmware_path)
            logger.info(f"Firmware downloaded to {firmware_path}")
            return str(firmware_path)
            
        except Exception as e:
            logger.error(f"Failed to download firmware: {e}")
            return None
    
    def flash_firmware(self, port: str, firmware_path: str = None, 
                      firmware_type: str = 'stable') -> bool:
        """Flash ESP-CSI firmware to ESP32"""
        if not firmware_path:
            firmware_path = self.download_firmware(firmware_type)
            if not firmware_path:
                return False
        
        if not os.path.exists(firmware_path):
            logger.error(f"Firmware file not found: {firmware_path}")
            return False
        
        logger.info(f"Flashing firmware to {port}")
        
        try:
            # Use esptool to flash firmware
            cmd = [
                'esptool.py',
                '--port', port,
                '--baud', '921600',
                '--before', 'default_reset',
                '--after', 'hard_reset',
                'write_flash',
                '--flash_mode', 'dio',
                '--flash_freq', '40m',
                '--flash_size', 'detect',
                '0x1000', firmware_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            
            if result.returncode == 0:
                logger.info("Firmware flashed successfully")
                return True
            else:
                logger.error(f"Flash failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("Flash operation timed out")
            return False
        except Exception as e:
            logger.error(f"Flash operation failed: {e}")
            return False
    
    def configure_device(self, port: str, wifi_ssid: str, wifi_password: str = "",
                        device_mode: str = "RX", csi_rate: int = 10, channel: int = 6, 
                        bandwidth: int = 20, tx_power: int = 15, beacon_interval: int = 100,
                        filter_mac: str = "", debug: bool = False) -> bool:
        """Configure ESP32 device with WiFi and CSI settings"""
        logger.info(f"Configuring device on {port} in {device_mode} mode")
        
        # Create configuration JSON
        config_json = self.ESP_CSI_CONFIG_TEMPLATE.format(
            ssid=wifi_ssid,
            password=wifi_password,
            device_mode=device_mode,
            csi_rate=csi_rate,
            channel=channel,
            bandwidth=bandwidth,
            tx_power=tx_power,
            beacon_interval=beacon_interval,
            filter_mac=filter_mac,
            debug=str(debug).lower()
        )
        
        try:
            # Connect to device
            ser = serial.Serial(port, 115200, timeout=10)
            time.sleep(2)  # Wait for device to settle
            
            # Send configuration command
            config_cmd = f"CONFIG:{config_json}\n"
            ser.write(config_cmd.encode())
            
            # Wait for acknowledgment
            response = ser.readline().decode().strip()
            ser.close()
            
            if "OK" in response:
                logger.info("Device configured successfully")
                return True
            else:
                logger.error(f"Configuration failed: {response}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to configure device: {e}")
            return False
    
    def test_device_connection(self, port: str, duration: float = 5.0) -> Dict[str, any]:
        """Test ESP32 device connection and data flow"""
        logger.info(f"Testing device connection on {port}")
        
        test_results = {
            'connection': False,
            'data_flow': False,
            'packets_received': 0,
            'test_duration': duration,
            'error': None
        }
        
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            test_results['connection'] = True
            
            # Send test command
            ser.write(b"TEST\n")
            
            start_time = time.time()
            packet_count = 0
            
            while time.time() - start_time < duration:
                if ser.in_waiting > 0:
                    data = ser.readline()
                    if data:
                        packet_count += 1
                
                time.sleep(0.1)
            
            ser.close()
            
            test_results['packets_received'] = packet_count
            test_results['data_flow'] = packet_count > 0
            
            logger.info(f"Test completed: {packet_count} packets received")
            
        except Exception as e:
            test_results['error'] = str(e)
            logger.error(f"Test failed: {e}")
        
        return test_results
    
    def setup_device_complete(self, port: str, firmware_type: str = 'stable',
                             wifi_ssid: str = "", wifi_password: str = "",
                             device_mode: str = "RX", csi_rate: int = 10, channel: int = 6,
                             tx_power: int = 15) -> bool:
        """Complete device setup: flash firmware and configure"""
        logger.info(f"Starting complete setup for device on {port} in {device_mode} mode")
        
        # Step 1: Flash firmware
        if not self.flash_firmware(port, firmware_type=firmware_type):
            logger.error("Failed to flash firmware")
            return False
        
        # Wait for device to boot
        time.sleep(5)
        
        # Step 2: Configure device
        if not self.configure_device(port, wifi_ssid, wifi_password, device_mode, 
                                    csi_rate, channel, tx_power=tx_power):
            logger.error("Failed to configure device")
            return False
        
        # Step 3: Test connection
        test_results = self.test_device_connection(port)
        if not test_results['connection']:
            logger.error("Device connection test failed")
            return False
        
        logger.info("Device setup completed successfully")
        return True
    
    def create_device_config(self, device_id: str, port: str, name: str = "",
                            location: str = "", device_mode: str = "RX") -> Dict[str, any]:
        """Create device configuration entry"""
        return {
            'id': device_id,
            'name': name or f"ESP32 Device {device_id}",
            'port': port,
            'location': location,
            'device_mode': device_mode,
            'setup_time': time.time(),
            'enabled': True,
            'calibration_offset': 0.0,
            'role': 'anchor' if device_mode == 'RX' else 'mobile'
        }


def main():
    """Main entry point for ESP32 setup tool"""
    parser = argparse.ArgumentParser(description='ESP32 Setup and Configuration Tool')
    
    # Subcommands
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Scan command
    scan_parser = subparsers.add_parser('scan', help='Scan for ESP32 devices')
    
    # Flash command
    flash_parser = subparsers.add_parser('flash', help='Flash ESP-CSI firmware')
    flash_parser.add_argument('--port', required=True, help='Serial port')
    flash_parser.add_argument('--firmware', choices=['stable', 'latest'], 
                             default='stable', help='Firmware type')
    flash_parser.add_argument('--firmware-file', help='Custom firmware file path')
    
    # Configure command
    config_parser = subparsers.add_parser('configure', help='Configure ESP32 device')
    config_parser.add_argument('--port', required=True, help='Serial port')
    config_parser.add_argument('--ssid', required=True, help='WiFi SSID')
    config_parser.add_argument('--password', default='', help='WiFi password')
    config_parser.add_argument('--rate', type=int, default=10, help='CSI sampling rate')
    config_parser.add_argument('--channel', type=int, default=6, help='WiFi channel')
    
    # Test command
    test_parser = subparsers.add_parser('test', help='Test device connection')
    test_parser.add_argument('--port', required=True, help='Serial port')
    test_parser.add_argument('--duration', type=float, default=5.0, help='Test duration')
    
    # Setup command (complete setup)
    setup_parser = subparsers.add_parser('setup', help='Complete device setup')
    setup_parser.add_argument('--port', required=True, help='Serial port')
    setup_parser.add_argument('--ssid', required=True, help='WiFi SSID')
    setup_parser.add_argument('--password', default='', help='WiFi password')
    setup_parser.add_argument('--firmware', choices=['stable', 'latest'], 
                             default='stable', help='Firmware type')
    
    # Global options
    parser.add_argument('--config', help='Configuration file path')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    if not args.command:
        parser.print_help()
        return 1
    
    # Initialize setup manager
    setup_manager = ESP32SetupManager(args.config)
    
    try:
        if args.command == 'scan':
            devices = setup_manager.scan_devices()
            print(f"\nFound {len(devices)} serial devices:")
            for device in devices:
                esp32_indicator = " [ESP32?]" if device['is_esp32'] else ""
                print(f"  {device['device']} - {device['description']}{esp32_indicator}")
        
        elif args.command == 'flash':
            firmware_path = args.firmware_file
            success = setup_manager.flash_firmware(args.port, firmware_path, args.firmware)
            return 0 if success else 1
        
        elif args.command == 'configure':
            success = setup_manager.configure_device(
                args.port, args.ssid, args.password, args.rate, args.channel
            )
            return 0 if success else 1
        
        elif args.command == 'test':
            results = setup_manager.test_device_connection(args.port, args.duration)
            print(f"Connection: {'OK' if results['connection'] else 'FAILED'}")
            print(f"Data flow: {'OK' if results['data_flow'] else 'FAILED'}")
            print(f"Packets received: {results['packets_received']}")
            return 0 if results['connection'] else 1
        
        elif args.command == 'setup':
            success = setup_manager.setup_device_complete(
                args.port, args.firmware, args.ssid, args.password
            )
            return 0 if success else 1
    
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        return 1
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())