"""
Configuration Manager for CSI Collector
Handles loading and validation of YAML configuration files
"""

import os
import yaml
import logging
from pathlib import Path
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class ConfigManager:
    """Manages configuration loading and validation"""
    
    DEFAULT_CONFIG = {
        'serial': {
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'timeout': 5.0,
            'auto_reconnect': True,
            'reconnect_delay': 5.0
        },
        'data': {
            'output_dir': '/app/data',
            'format': 'csv',
            'max_file_size': 100,  # MB
            'compression': True,
            'buffer_size': 1000,
            'flush_interval': 30.0  # seconds
        },
        'ros2': {
            'enabled': False,
            'domain_id': 0,
            'topic': '/csi_data',
            'qos_profile': 'default'
        },
        'esp32': {
            'wifi_ssid': '',
            'wifi_password': '',
            'csi_rate': 10,  # Hz
            'channel': 6,
            'bandwidth': 20,  # MHz
            'filter_mac': '',
            'enable_debug': False
        },
        'api': {
            'host': '0.0.0.0',
            'port': 5000,
            'cors_origins': ['*'],
            'max_connections': 100
        },
        'logging': {
            'level': 'INFO',
            'file_path': '/app/logs/csi_collector.log',
            'max_file_size': 10,  # MB
            'backup_count': 5
        }
    }
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self._config = None
        self.load_config()
    
    def load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file"""
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    file_config = yaml.safe_load(f) or {}
                
                # Merge with defaults
                self._config = self._deep_merge(self.DEFAULT_CONFIG.copy(), file_config)
                logger.info(f"Configuration loaded from {self.config_path}")
            else:
                logger.warning(f"Config file not found: {self.config_path}, using defaults")
                self._config = self.DEFAULT_CONFIG.copy()
                
            self.validate_config()
            return self._config
            
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            logger.info("Using default configuration")
            self._config = self.DEFAULT_CONFIG.copy()
            return self._config
    
    def get_config(self) -> Dict[str, Any]:
        """Get the current configuration"""
        if self._config is None:
            self.load_config()
        return self._config
    
    def save_config(self, config: Dict[str, Any]) -> bool:
        """Save configuration to file"""
        try:
            # Validate before saving
            self._validate_config_dict(config)
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)
            
            self._config = config
            logger.info(f"Configuration saved to {self.config_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save configuration: {e}")
            return False
    
    def validate_config(self):
        """Validate the current configuration"""
        self._validate_config_dict(self._config)
    
    def _validate_config_dict(self, config: Dict[str, Any]):
        """Validate configuration dictionary"""
        # Validate serial settings
        serial_config = config.get('serial', {})
        if not isinstance(serial_config.get('baudrate'), int):
            raise ValueError("Serial baudrate must be an integer")
        
        if serial_config.get('baudrate', 0) <= 0:
            raise ValueError("Serial baudrate must be positive")
        
        # Validate data settings
        data_config = config.get('data', {})
        if not isinstance(data_config.get('max_file_size'), (int, float)):
            raise ValueError("Data max_file_size must be a number")
        
        if data_config.get('max_file_size', 0) <= 0:
            raise ValueError("Data max_file_size must be positive")
        
        # Validate ROS2 settings
        ros2_config = config.get('ros2', {})
        if ros2_config.get('enabled', False):
            domain_id = ros2_config.get('domain_id', 0)
            if not isinstance(domain_id, int) or domain_id < 0 or domain_id > 101:
                raise ValueError("ROS2 domain_id must be an integer between 0 and 101")
        
        # Validate ESP32 settings
        esp32_config = config.get('esp32', {})
        csi_rate = esp32_config.get('csi_rate', 10)
        if not isinstance(csi_rate, int) or csi_rate <= 0 or csi_rate > 100:
            raise ValueError("ESP32 csi_rate must be an integer between 1 and 100")
        
        channel = esp32_config.get('channel', 6)
        if not isinstance(channel, int) or channel < 1 or channel > 13:
            raise ValueError("ESP32 channel must be an integer between 1 and 13")
    
    def _deep_merge(self, base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
        """Deep merge two dictionaries"""
        result = base.copy()
        
        for key, value in overlay.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = value
        
        return result
    
    @staticmethod
    def create_default_config(config_path: str):
        """Create a default configuration file"""
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            
            with open(config_path, 'w') as f:
                yaml.dump(ConfigManager.DEFAULT_CONFIG, f, default_flow_style=False, indent=2)
            
            logger.info(f"Default configuration created at {config_path}")
            
        except Exception as e:
            logger.error(f"Failed to create default configuration: {e}")
            raise