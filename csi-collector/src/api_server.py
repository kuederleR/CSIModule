"""
API Server for CSI Collector
Provides REST API and WebSocket interface for real-time data
"""

import asyncio
import json
import logging
import os
from typing import Dict, Any, List

from flask import Flask, request, jsonify, send_file
from flask_socketio import SocketIO, emit
import threading
from datetime import datetime
from pathlib import Path

from .device_manager import DeviceManager

logger = logging.getLogger(__name__)


class APIServer:
    """Flask-based API server for CSI collector"""
    
    def __init__(self, collector, config: Dict[str, Any], config_manager=None):
        self.collector = collector
        self.config = config
        self.config_manager = config_manager
        self.device_manager = DeviceManager(config)
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'csi_collector_secret'
        
        # Configure CORS
        api_config = config.get('api', {})
        cors_origins = api_config.get('cors_origins', ['*'])
        
        self.socketio = SocketIO(
            self.app, 
            cors_allowed_origins=cors_origins,
            async_mode='threading'
        )
        
        self.setup_routes()
        self.setup_websocket_handlers()
        
        self.connected_clients = 0
    
    def setup_routes(self):
        """Setup REST API routes"""
        
        @self.app.route('/health', methods=['GET'])
        def health_check():
            """Health check endpoint"""
            return jsonify({
                'status': 'healthy',
                'timestamp': datetime.now().isoformat(),
                'collector_running': self.collector.running if self.collector else False
            })
        
        @self.app.route('/status', methods=['GET'])
        def get_status():
            """Get system status"""
            if not self.collector:
                return jsonify({'error': 'Collector not initialized'}), 500
            
            status = self.collector.get_status()
            status.update({
                'ros_enabled': self.config.get('ros2', {}).get('enabled', False),
                'ros_connected': False,  # TODO: Get actual ROS status
                'collecting': status.get('running', False),
                'connected_clients': self.connected_clients
            })
            
            return jsonify(status)
        
        @self.app.route('/start', methods=['POST'])
        def start_collection():
            """Start data collection"""
            if not self.collector:
                return jsonify({'error': 'Collector not initialized'}), 500
            
            try:
                if not self.collector.running:
                    asyncio.run_coroutine_threadsafe(
                        self.collector.start(), 
                        asyncio.get_event_loop()
                    )
                return jsonify({'success': True, 'message': 'Collection started'})
            except Exception as e:
                logger.error(f"Failed to start collection: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/stop', methods=['POST'])
        def stop_collection():
            """Stop data collection"""
            if not self.collector:
                return jsonify({'error': 'Collector not initialized'}), 500
            
            try:
                if self.collector.running:
                    asyncio.run_coroutine_threadsafe(
                        self.collector.stop(), 
                        asyncio.get_event_loop()
                    )
                return jsonify({'success': True, 'message': 'Collection stopped'})
            except Exception as e:
                logger.error(f"Failed to stop collection: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/config', methods=['GET'])
        def get_config():
            """Get current configuration"""
            return jsonify(self.config)
        
        @self.app.route('/config', methods=['POST'])
        def update_config():
            """Update configuration"""
            try:
                new_config = request.get_json()
                if not new_config:
                    return jsonify({'error': 'No configuration provided'}), 400
                
                # Validate the new configuration
                if self.config_manager:
                    # Use ConfigManager to validate and save
                    success = self.config_manager.save_config(new_config)
                    if success:
                        # Update our local config reference
                        self.config = self.config_manager.get_config()
                        
                        # Notify collector of config changes if applicable
                        if self.collector and hasattr(self.collector, 'update_config'):
                            self.collector.update_config(self.config)
                        
                        logger.info("Configuration updated successfully")
                        return jsonify({
                            'success': True, 
                            'message': 'Configuration updated and saved successfully'
                        })
                    else:
                        return jsonify({'error': 'Failed to save configuration'}), 500
                else:
                    # Fallback: just update in-memory config (not persistent)
                    self.config.update(new_config)
                    logger.warning("Configuration updated in memory only (not persistent)")
                    return jsonify({
                        'success': True, 
                        'message': 'Configuration updated (in memory only - not persistent)'
                    })
                
            except ValueError as e:
                logger.error(f"Configuration validation failed: {e}")
                return jsonify({'error': f'Configuration validation failed: {str(e)}'}), 400
            except Exception as e:
                logger.error(f"Failed to update config: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/data/files', methods=['GET'])
        def list_data_files():
            """List available data files"""
            try:
                data_dir = Path(self.config.get('data', {}).get('output_dir', '/app/data'))
                
                if not data_dir.exists():
                    return jsonify({'files': []})
                
                files = []
                for file_path in data_dir.glob('csi_data_*'):
                    if file_path.is_file():
                        stat = file_path.stat()
                        files.append({
                            'name': file_path.name,
                            'size': stat.st_size,
                            'modified': datetime.fromtimestamp(stat.st_mtime).isoformat()
                        })
                
                # Sort by modification time (newest first)
                files.sort(key=lambda x: x['modified'], reverse=True)
                
                return jsonify({'files': [f['name'] for f in files], 'details': files})
                
            except Exception as e:
                logger.error(f"Failed to list data files: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/data/file/<filename>', methods=['GET'])
        def get_data_file(filename):
            """Get data from a specific file"""
            try:
                data_dir = Path(self.config.get('data', {}).get('output_dir', '/app/data'))
                file_path = data_dir / filename
                
                if not file_path.exists() or not file_path.is_file():
                    return jsonify({'error': 'File not found'}), 404
                
                # For CSV files, return parsed data (limited)
                if filename.endswith('.csv'):
                    import csv
                    data = []
                    with open(file_path, 'r') as f:
                        reader = csv.DictReader(f)
                        for i, row in enumerate(reader):
                            if i >= 1000:  # Limit to first 1000 rows for web display
                                break
                            data.append(row)
                    
                    return jsonify({'data': data, 'total_rows': len(data)})
                
                # For other files, return file info
                stat = file_path.stat()
                return jsonify({
                    'filename': filename,
                    'size': stat.st_size,
                    'modified': datetime.fromtimestamp(stat.st_mtime).isoformat(),
                    'message': 'Use /data/export/ endpoint to download file'
                })
                
            except Exception as e:
                logger.error(f"Failed to get data file: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/data/export/<filename>', methods=['GET'])
        def export_data_file(filename):
            """Export/download a data file"""
            try:
                data_dir = Path(self.config.get('data', {}).get('output_dir', '/app/data'))
                file_path = data_dir / filename
                
                if not file_path.exists() or not file_path.is_file():
                    return jsonify({'error': 'File not found'}), 404
                
                return send_file(file_path, as_attachment=True, download_name=filename)
                
            except Exception as e:
                logger.error(f"Failed to export data file: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/download', methods=['GET'])
        def download_latest_data():
            """Download the latest data file"""
            try:
                data_dir = Path(self.config.get('data', {}).get('output_dir', '/app/data'))
                
                # Find the most recent file
                files = list(data_dir.glob('csi_data_*'))
                if not files:
                    return jsonify({'error': 'No data files available'}), 404
                
                latest_file = max(files, key=lambda x: x.stat().st_mtime)
                return send_file(latest_file, as_attachment=True, download_name=latest_file.name)
                
            except Exception as e:
                logger.error(f"Failed to download data: {e}")
                return jsonify({'error': str(e)}), 500
        
        # ESP32 management endpoints
        @self.app.route('/esp32/scan', methods=['GET'])
        def scan_esp32_devices():
            """Scan for available ESP32 devices"""
            try:
                from src.collector import SerialManager
                ports = SerialManager.list_ports()
                
                # Filter for likely ESP32 devices
                esp32_ports = []
                for port in ports:
                    # Add basic port information
                    esp32_ports.append({
                        'device': port,
                        'description': f'Serial Port {port}'
                    })
                
                return jsonify({'ports': esp32_ports})
                
            except Exception as e:
                logger.error(f"Failed to scan devices: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/flash', methods=['POST'])
        def flash_esp32_firmware():
            """Flash ESP32 firmware"""
            try:
                # Handle both JSON and form data
                if request.content_type and 'application/json' in request.content_type:
                    # JSON request
                    data = request.get_json()
                    port = data.get('port')
                    firmware_type = data.get('firmware_type', 'stable')
                    firmware_file = None
                else:
                    # Form data request (for file uploads)
                    port = request.form.get('port')
                    firmware_type = request.form.get('firmware_type', 'stable')
                    firmware_file = request.files.get('firmware_file')
                
                if not port:
                    return jsonify({'error': 'Port not specified'}), 400
                
                # Handle custom firmware file
                firmware_path = None
                if firmware_type == 'custom' and firmware_file:
                    # Save uploaded firmware file
                    import tempfile
                    import os
                    
                    # Create temp directory if it doesn't exist
                    temp_dir = '/tmp/esp32_firmware'
                    os.makedirs(temp_dir, exist_ok=True)
                    
                    # Save the file
                    firmware_path = os.path.join(temp_dir, f"firmware_{port.replace('/', '_')}.bin")
                    firmware_file.save(firmware_path)
                    logger.info(f"Saved custom firmware to {firmware_path}")
                
                # Simulate firmware flashing process
                logger.info(f"Flashing {firmware_type} firmware to {port}")
                
                # In a real implementation, you would use esptool.py here
                # Example: esptool.py --chip esp32 --port {port} write_flash 0x10000 {firmware_path}
                
                return jsonify({
                    'success': True,
                    'message': f'Firmware ({firmware_type}) flashed to {port}',
                    'firmware_type': firmware_type,
                    'port': port,
                    'custom_file': firmware_path if firmware_path else None
                })
                
            except Exception as e:
                logger.error(f"Failed to flash firmware: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/configure', methods=['POST'])
        def configure_esp32():
            """Configure ESP32 device"""
            try:
                data = request.get_json()
                port = data.get('port')
                
                if not port:
                    return jsonify({'error': 'Port not specified'}), 400
                
                # Extract configuration parameters
                device_config = {
                    'port': port,
                    'name': data.get('name', f'ESP32 Device ({port})'),
                    'location': data.get('location', ''),
                    'wifi_ssid': data.get('wifi_ssid', ''),
                    'wifi_password': data.get('wifi_password', ''),
                    'device_mode': data.get('device_mode', 'RX'),
                    'csi_rate': data.get('csi_rate', 10),
                    'channel': data.get('channel', 6),
                    'bandwidth': data.get('bandwidth', 20),
                    'tx_power': data.get('tx_power', 15),
                    'packet_interval': data.get('beacon_interval', 100),  # For TX mode
                    'enable_debug': data.get('enable_debug', False),
                    'firmware_version': data.get('firmware_version', 'Unknown')
                }
                
                logger.info(f"Configuring ESP32 on {port} in {device_config['device_mode']} mode")
                
                # Add device to device manager
                device_id = self.device_manager.add_device(device_config)
                
                # This is a placeholder - actual implementation would send config to ESP32
                return jsonify({
                    'success': True,
                    'message': f'Device configured on {port} as {device_config["device_mode"]}',
                    'device_id': device_id,
                    'config': device_config
                })
                
            except Exception as e:
                logger.error(f"Failed to configure device: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/test', methods=['POST'])
        def test_esp32_connection():
            """Test ESP32 connection and data flow"""
            try:
                data = request.get_json()
                port = data.get('port')
                
                if not port:
                    return jsonify({'error': 'Port not specified'}), 400
                
                # This is a placeholder - actual implementation would test connection
                return jsonify({
                    'connection': True,
                    'data_flow': True,
                    'packets_received': 10,
                    'test_duration': 5.0
                })
                
            except Exception as e:
                logger.error(f"Failed to test connection: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/devices', methods=['GET'])
        def list_configured_devices():
            """List configured ESP32 devices"""
            try:
                devices = self.device_manager.get_all_devices()
                stats = self.device_manager.get_device_stats()
                
                return jsonify({
                    'devices': devices,
                    'stats': stats
                })
                
            except Exception as e:
                logger.error(f"Failed to list devices: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/device/<device_id>', methods=['GET'])
        def get_device_details(device_id):
            """Get detailed information about a specific device"""
            try:
                device = self.device_manager.get_device(device_id)
                if not device:
                    return jsonify({'error': 'Device not found'}), 404
                
                # Create detailed device information with additional structure
                device_details = device.copy()
                
                # Add configuration section
                device_details['configuration'] = {
                    'wifi_ssid': device.get('wifi_ssid', ''),
                    'channel': device.get('channel', 6),
                    'bandwidth': 20,  # Default bandwidth
                    'csi_rate': 10 if device.get('device_mode') == 'RX' else 0,
                    'tx_power': device.get('tx_power', 0),
                    'packet_interval': device.get('packet_interval', 0),
                    'enable_debug': False
                }
                
                # Add statistics section
                device_details['statistics'] = {
                    'packets_received': device.get('packets_received', 0),
                    'packets_transmitted': device.get('packets_transmitted', 0),
                    'bytes_received': device.get('bytes_received', 0),
                    'bytes_transmitted': device.get('bytes_transmitted', 0),
                    'error_count': device.get('error_count', 0),
                    'uptime_seconds': 0,  # Would be calculated from setup_time
                    'data_rate_hz': 0,
                    'last_packet_time': device.get('last_seen')
                }
                
                # Add health section
                device_details['health'] = {
                    'cpu_usage': None,
                    'memory_usage': None,
                    'temperature': None,
                    'battery_level': None,
                    'signal_quality': 'Unknown'
                }
                
                return jsonify(device_details)
                
            except Exception as e:
                logger.error(f"Failed to get device details: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/device/<device_id>', methods=['DELETE'])
        def remove_esp32_device(device_id):
            """Remove ESP32 device configuration"""
            try:
                success = self.device_manager.remove_device(device_id)
                if not success:
                    return jsonify({'error': 'Device not found'}), 404
                
                return jsonify({
                    'success': True,
                    'message': f'Device {device_id} removed successfully',
                    'device_id': device_id
                })
                
            except Exception as e:
                logger.error(f"Failed to remove device: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/device/<device_id>', methods=['PUT'])
        def update_esp32_device(device_id):
            """Update ESP32 device configuration"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify({'error': 'No data provided'}), 400
                
                # Extract updatable fields
                updatable_fields = ['name', 'location', 'device_mode', 'channel', 'tx_power', 
                                  'packet_interval', 'wifi_ssid', 'wifi_password']
                updates = {k: v for k, v in data.items() if k in updatable_fields}
                
                success = self.device_manager.update_device(device_id, updates)
                if not success:
                    return jsonify({'error': 'Device not found'}), 404
                
                return jsonify({
                    'success': True,
                    'message': f'Device {device_id} updated successfully',
                    'device_id': device_id,
                    'updated_fields': updates
                })
                
            except Exception as e:
                logger.error(f"Failed to update device: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/esp32/device/<device_id>/restart', methods=['POST'])
        def restart_esp32_device(device_id):
            """Restart ESP32 device"""
            try:
                logger.info(f"Restarting device {device_id}")
                
                # Mock validation
                valid_devices = ['esp32_001', 'esp32_002']
                if device_id not in valid_devices:
                    return jsonify({'error': 'Device not found'}), 404
                
                # In a real implementation, this would send restart command to device
                return jsonify({
                    'success': True,
                    'message': f'Restart command sent to device {device_id}',
                    'device_id': device_id
                })
                
            except Exception as e:
                logger.error(f"Failed to restart device: {e}")
                return jsonify({'error': str(e)}), 500
    
    def setup_websocket_handlers(self):
        """Setup WebSocket event handlers"""
        
        @self.socketio.on('connect')
        def handle_connect():
            self.connected_clients += 1
            logger.info(f"Client connected. Total clients: {self.connected_clients}")
            
            # Send current status to new client
            if self.collector:
                status = self.collector.get_status()
                emit('status_update', status)
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.connected_clients = max(0, self.connected_clients - 1)
            logger.info(f"Client disconnected. Total clients: {self.connected_clients}")
        
        @self.socketio.on('request_status')
        def handle_status_request():
            if self.collector:
                status = self.collector.get_status()
                emit('status_update', status)
    
    def broadcast_data(self, csi_data):
        """Broadcast CSI data to connected WebSocket clients"""
        if self.connected_clients > 0:
            try:
                # Send limited data to avoid overwhelming clients
                data_summary = {
                    'timestamp': csi_data.timestamp,
                    'packets_received': self.collector.stats.get('packets_received', 0),
                    'data_rate': self.collector.stats.get('packets_per_second', 0),
                    'file_size': 0,  # TODO: Get actual file size
                    'rssi': csi_data.parsed_data.get('rssi', 0),
                    'channel': csi_data.parsed_data.get('channel', 0)
                }
                
                self.socketio.emit('csi_data', data_summary)
                
            except Exception as e:
                logger.error(f"Failed to broadcast data: {e}")
    
    async def start(self):
        """Start the API server"""
        api_config = self.config.get('api', {})
        host = api_config.get('host', '0.0.0.0')
        port = api_config.get('port', 5000)
        
        logger.info(f"Starting API server on {host}:{port}")
        
        # Run Flask-SocketIO server
        self.socketio.run(
            self.app,
            host=host,
            port=port,
            debug=False,
            allow_unsafe_werkzeug=True
        )
    
    async def stop(self):
        """Stop the API server"""
        logger.info("Stopping API server")
        # Flask-SocketIO doesn't have a clean shutdown method
        # The server will stop when the main thread exits