"""
Device Manager for CSI Collector
Manages ESP32 device configurations and persistent storage
"""

import json
import logging
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any

logger = logging.getLogger(__name__)


class DeviceManager:
    """Manages ESP32 device configurations and storage"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.devices_file = Path('/app/data/configured_devices.json')
        self._devices = {}
        self.load_devices()
    
    def load_devices(self):
        """Load devices from persistent storage"""
        try:
            if self.devices_file.exists():
                with open(self.devices_file, 'r') as f:
                    self._devices = json.load(f)
                logger.info(f"Loaded {len(self._devices)} configured devices")
            else:
                self._devices = {}
                logger.info("No existing device configuration found, starting with empty device list")
        except Exception as e:
            logger.error(f"Failed to load devices: {e}")
            self._devices = {}
    
    def save_devices(self):
        """Save devices to persistent storage"""
        try:
            # Ensure directory exists
            self.devices_file.parent.mkdir(parents=True, exist_ok=True)
            
            with open(self.devices_file, 'w') as f:
                json.dump(self._devices, f, indent=2)
            logger.info(f"Saved {len(self._devices)} configured devices")
            return True
        except Exception as e:
            logger.error(f"Failed to save devices: {e}")
            return False
    
    def add_device(self, device_config: Dict[str, Any]) -> str:
        """Add a new device configuration"""
        try:
            # Generate device ID
            device_id = f"esp32_{int(time.time())}"
            
            # Prepare device data
            device_data = {
                'id': device_id,
                'name': device_config.get('name', f'ESP32 Device {device_id}'),
                'port': device_config.get('port'),
                'device_mode': device_config.get('device_mode', 'RX'),
                'location': device_config.get('location', ''),
                'channel': device_config.get('channel', 6),
                'tx_power': device_config.get('tx_power', 20) if device_config.get('device_mode') == 'TX' else 0,
                'packet_interval': device_config.get('packet_interval', 100) if device_config.get('device_mode') == 'TX' else 0,
                'wifi_ssid': device_config.get('wifi_ssid', ''),
                'wifi_password': device_config.get('wifi_password', ''),
                'setup_time': datetime.now().isoformat(),
                'last_seen': None,
                'status': 'disconnected',
                'firmware_version': device_config.get('firmware_version', 'Unknown'),
                'hardware_revision': device_config.get('hardware_revision', 'Unknown'),
                'mac_address': device_config.get('mac_address', 'Unknown'),
                'uptime': '0s',
                'signal_strength': 0,
                'packets_received': 0,
                'packets_transmitted': 0,
                'bytes_received': 0,
                'bytes_transmitted': 0,
                'error_count': 0
            }
            
            self._devices[device_id] = device_data
            self.save_devices()
            
            logger.info(f"Added new device: {device_id} ({device_data['name']})")
            return device_id
            
        except Exception as e:
            logger.error(f"Failed to add device: {e}")
            raise
    
    def remove_device(self, device_id: str) -> bool:
        """Remove a device configuration"""
        try:
            if device_id in self._devices:
                device_name = self._devices[device_id].get('name', device_id)
                del self._devices[device_id]
                self.save_devices()
                logger.info(f"Removed device: {device_id} ({device_name})")
                return True
            else:
                logger.warning(f"Device not found: {device_id}")
                return False
        except Exception as e:
            logger.error(f"Failed to remove device: {e}")
            return False
    
    def update_device(self, device_id: str, updates: Dict[str, Any]) -> bool:
        """Update device configuration"""
        try:
            if device_id not in self._devices:
                logger.warning(f"Device not found: {device_id}")
                return False
            
            # Update device data
            self._devices[device_id].update(updates)
            self.save_devices()
            
            logger.info(f"Updated device: {device_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to update device: {e}")
            return False
    
    def get_device(self, device_id: str) -> Optional[Dict[str, Any]]:
        """Get device configuration by ID"""
        return self._devices.get(device_id)
    
    def get_all_devices(self) -> List[Dict[str, Any]]:
        """Get all device configurations"""
        return list(self._devices.values())
    
    def get_devices_by_mode(self, mode: str) -> List[Dict[str, Any]]:
        """Get devices filtered by mode (RX/TX)"""
        return [device for device in self._devices.values() if device.get('device_mode') == mode]
    
    def get_device_stats(self) -> Dict[str, int]:
        """Get device statistics"""
        devices = list(self._devices.values())
        connected_devices = [d for d in devices if d.get('status') == 'connected']
        rx_devices = [d for d in devices if d.get('device_mode') == 'RX']
        tx_devices = [d for d in devices if d.get('device_mode') == 'TX']
        
        return {
            'total_count': len(devices),
            'connected_count': len(connected_devices),
            'rx_count': len(rx_devices),
            'tx_count': len(tx_devices)
        }
    
    def update_device_status(self, device_id: str, status: str, last_seen: str = None):
        """Update device status and last seen time"""
        if device_id in self._devices:
            self._devices[device_id]['status'] = status
            if last_seen:
                self._devices[device_id]['last_seen'] = last_seen
            # Don't save on every status update to avoid excessive I/O
            # Status updates are typically frequent and ephemeral
    
    def update_device_stats(self, device_id: str, stats: Dict[str, Any]):
        """Update device statistics"""
        if device_id in self._devices:
            device = self._devices[device_id]
            
            # Update statistics
            for key, value in stats.items():
                if key in ['packets_received', 'packets_transmitted', 'bytes_received', 
                          'bytes_transmitted', 'error_count', 'signal_strength', 'uptime']:
                    device[key] = value
            
            # Update last seen
            device['last_seen'] = datetime.now().isoformat()
            
            # Save periodically (every 10th update to balance persistence and performance)
            if device.get('packets_received', 0) % 10 == 0:
                self.save_devices()