"""
CSI Data Collector
Handles serial communication with ESP32 devices and data processing
"""

import asyncio
import csv
import json
import logging
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Callable, Optional, Any
import struct

import serial
import serial.tools.list_ports
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

logger = logging.getLogger(__name__)


class CSIData:
    """Represents a single CSI measurement"""
    
    def __init__(self, raw_data: bytes, timestamp: float = None):
        self.timestamp = timestamp or time.time()
        self.raw_data = raw_data
        self.parsed_data = self._parse_data(raw_data)
    
    def _parse_data(self, data: bytes) -> Dict[str, Any]:
        """Parse CSI data from ESP32 format"""
        try:
            # ESP-CSI data structure parsing
            # This is a simplified parser - adjust based on actual ESP-CSI format
            if len(data) < 24:  # Minimum header size
                return {'error': 'Data too short'}
            
            # Parse header (simplified structure)
            header = struct.unpack('<IIHBBBB', data[:16])
            
            parsed = {
                'timestamp': self.timestamp,
                'packet_type': header[0],
                'length': header[1],
                'channel': header[2],
                'rssi': header[3] - 256 if header[3] > 127 else header[3],  # Convert to signed
                'rate': header[4],
                'bandwidth': header[5],
                'smoothing': header[6],
                'not_sounding': header[7],
                'aggregation': bool(header[7] & 0x01),
                'stbc': (header[7] >> 1) & 0x03,
                'fec_coding': bool(header[7] & 0x08),
                'sgi': bool(header[7] & 0x10),
                'noise_floor': -95,  # Default noise floor
                'amplitude': [],
                'phase': [],
                'data_length': len(data)
            }
            
            # Parse CSI data (amplitude and phase)
            csi_data = data[16:]
            if len(csi_data) >= 4:  # At least one complex sample
                # Each complex sample is 2 bytes (real + imaginary)
                num_samples = len(csi_data) // 2
                for i in range(0, min(num_samples, 64) * 2, 2):  # Limit to 64 subcarriers
                    if i + 1 < len(csi_data):
                        real = struct.unpack('<b', csi_data[i:i+1])[0]
                        imag = struct.unpack('<b', csi_data[i+1:i+2])[0]
                        
                        amplitude = (real**2 + imag**2)**0.5
                        phase = 0 if real == 0 and imag == 0 else \
                               (3.14159/2 if real == 0 else 
                                (3.14159/180) * (180/3.14159) * (imag/real if real != 0 else 0))
                        
                        parsed['amplitude'].append(amplitude)
                        parsed['phase'].append(phase)
            
            return parsed
            
        except Exception as e:
            logger.error(f"Failed to parse CSI data: {e}")
            return {
                'error': str(e),
                'timestamp': self.timestamp,
                'data_length': len(data)
            }
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary format"""
        return self.parsed_data
    
    def to_csv_row(self) -> List[str]:
        """Convert to CSV row format"""
        data = self.parsed_data
        return [
            str(data.get('timestamp', '')),
            str(data.get('channel', '')),
            str(data.get('rssi', '')),
            str(data.get('rate', '')),
            str(data.get('bandwidth', '')),
            str(len(data.get('amplitude', []))),
            json.dumps(data.get('amplitude', [])),
            json.dumps(data.get('phase', []))
        ]


class SerialManager:
    """Manages serial connection to ESP32 device"""
    
    def __init__(self, port: str, baudrate: int, timeout: float = 5.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection = None
        self.connected = False
    
    async def connect(self) -> bool:
        """Connect to serial device"""
        try:
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.connected = True
            logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            self.connected = False
            return False
    
    async def disconnect(self):
        """Disconnect from serial device"""
        if self.connection and self.connection.is_open:
            self.connection.close()
        self.connected = False
        logger.info(f"Disconnected from {self.port}")
    
    async def read_data(self) -> Optional[bytes]:
        """Read data from serial connection"""
        if not self.connected or not self.connection:
            return None
        
        try:
            if self.connection.in_waiting > 0:
                # Read available data
                data = self.connection.read(self.connection.in_waiting)
                return data
            return None
            
        except Exception as e:
            logger.error(f"Failed to read from serial: {e}")
            self.connected = False
            return None
    
    async def write_data(self, data: bytes) -> bool:
        """Write data to serial connection"""
        if not self.connected or not self.connection:
            return False
        
        try:
            self.connection.write(data)
            return True
            
        except Exception as e:
            logger.error(f"Failed to write to serial: {e}")
            return False
    
    @staticmethod
    def list_ports():
        """List available serial ports"""
        return [port.device for port in serial.tools.list_ports.comports()]


class DataWriter:
    """Handles writing CSI data to files"""
    
    def __init__(self, output_dir: str, format: str = 'csv', max_file_size: int = 100):
        self.output_dir = Path(output_dir)
        self.format = format
        self.max_file_size = max_file_size * 1024 * 1024  # Convert MB to bytes
        self.current_file = None
        self.current_writer = None
        self.records_written = 0
        
        # Ensure output directory exists
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    def _get_new_filename(self) -> str:
        """Generate new filename with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"csi_data_{timestamp}.{self.format}"
    
    def _open_new_file(self):
        """Open a new data file"""
        if self.current_file:
            self.current_file.close()
        
        filename = self._get_new_filename()
        filepath = self.output_dir / filename
        
        if self.format == 'csv':
            self.current_file = open(filepath, 'w', newline='')
            self.current_writer = csv.writer(self.current_file)
            # Write header
            self.current_writer.writerow([
                'timestamp', 'channel', 'rssi', 'rate', 'bandwidth', 
                'num_subcarriers', 'amplitude', 'phase'
            ])
        elif self.format == 'json':
            self.current_file = open(filepath, 'w')
            self.current_writer = None
        else:
            self.current_file = open(filepath, 'wb')
            self.current_writer = None
        
        self.records_written = 0
        logger.info(f"Opened new data file: {filepath}")
    
    def write_data(self, csi_data: CSIData):
        """Write CSI data to file"""
        # Check if we need a new file
        if (self.current_file is None or 
            (self.current_file.tell() > self.max_file_size)):
            self._open_new_file()
        
        try:
            if self.format == 'csv':
                self.current_writer.writerow(csi_data.to_csv_row())
            elif self.format == 'json':
                json.dump(csi_data.to_dict(), self.current_file)
                self.current_file.write('\n')
            else:  # binary
                self.current_file.write(csi_data.raw_data)
            
            self.records_written += 1
            
            # Flush periodically
            if self.records_written % 100 == 0:
                self.current_file.flush()
                
        except Exception as e:
            logger.error(f"Failed to write data: {e}")
    
    def close(self):
        """Close current file"""
        if self.current_file:
            self.current_file.close()
            self.current_file = None


class CSICollector:
    """Main CSI data collector class"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.serial_manager = None
        self.data_writer = None
        self.data_handlers = []
        self.running = False
        self.stats = {
            'packets_received': 0,
            'packets_per_second': 0,
            'bytes_received': 0,
            'start_time': None,
            'last_packet_time': None
        }
        self._data_buffer = []
        self._last_stats_update = time.time()
    
    def add_data_handler(self, handler: Callable[[CSIData], None]):
        """Add a data handler callback"""
        self.data_handlers.append(handler)
    
    async def start(self):
        """Start data collection"""
        if self.running:
            return
        
        serial_config = self.config.get('serial', {})
        data_config = self.config.get('data', {})
        
        # Initialize serial manager
        self.serial_manager = SerialManager(
            port=serial_config.get('port', '/dev/ttyUSB0'),
            baudrate=serial_config.get('baudrate', 115200),
            timeout=serial_config.get('timeout', 5.0)
        )
        
        # Initialize data writer
        self.data_writer = DataWriter(
            output_dir=data_config.get('output_dir', '/app/data'),
            format=data_config.get('format', 'csv'),
            max_file_size=data_config.get('max_file_size', 100)
        )
        
        # Try to connect to device (don't fail if no device available)
        connected = await self.serial_manager.connect()
        if not connected:
            logger.warning("No ESP32 device connected. Running in demo mode.")
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        logger.info("CSI data collection started")
        
        # Start data collection loop (only if device connected)
        if connected:
            asyncio.create_task(self._collection_loop())
        else:
            # Start demo data generation
            asyncio.create_task(self._demo_loop())
            
        asyncio.create_task(self._stats_loop())
    
    async def stop(self):
        """Stop data collection"""
        if not self.running:
            return
        
        self.running = False
        
        if self.serial_manager:
            await self.serial_manager.disconnect()
        
        if self.data_writer:
            self.data_writer.close()
        
        logger.info("CSI data collection stopped")
    
    async def _collection_loop(self):
        """Main data collection loop"""
        packet_buffer = b''
        
        while self.running:
            try:
                # Read data from serial
                data = await self.serial_manager.read_data()
                if data:
                    packet_buffer += data
                    
                    # Process complete packets
                    while len(packet_buffer) >= 4:  # Minimum packet size
                        # Simple packet framing - look for start marker
                        if packet_buffer.startswith(b'\xfe\xfe'):
                            # Found potential start of packet
                            if len(packet_buffer) < 6:
                                break  # Need more data for length
                            
                            # Get packet length
                            packet_length = struct.unpack('<H', packet_buffer[2:4])[0]
                            
                            if len(packet_buffer) >= packet_length + 4:
                                # Complete packet available
                                packet_data = packet_buffer[4:packet_length + 4]
                                packet_buffer = packet_buffer[packet_length + 4:]
                                
                                # Process packet
                                await self._process_packet(packet_data)
                            else:
                                break  # Need more data
                        else:
                            # No start marker, remove first byte and continue
                            packet_buffer = packet_buffer[1:]
                
                await asyncio.sleep(0.001)  # Small delay to prevent busy waiting
                
            except Exception as e:
                logger.error(f"Error in collection loop: {e}")
                await asyncio.sleep(1)
    
    async def _demo_loop(self):
        """Demo data generation loop when no ESP32 is connected"""
        logger.info("Starting demo data generation...")
        
        import random
        import math
        
        packet_count = 0
        while self.running:
            try:
                # Generate synthetic CSI data
                num_subcarriers = 64
                amplitudes = []
                phases = []
                
                # Generate realistic CSI data with some patterns
                base_freq = 0.1 * packet_count  # Slow variation
                for i in range(num_subcarriers):
                    # Amplitude with frequency response pattern
                    amp = 10 + 5 * math.cos(2 * math.pi * i / num_subcarriers) + random.uniform(-2, 2)
                    amplitudes.append(max(0, amp))
                    
                    # Phase with some coherent structure
                    phase = math.sin(2 * math.pi * i / num_subcarriers + base_freq) + random.uniform(-0.5, 0.5)
                    phases.append(phase)
                
                # Create synthetic packet data
                demo_data = {
                    'timestamp': time.time(),
                    'packet_type': 1,
                    'length': 200,
                    'channel': 6,
                    'rssi': -45 + random.uniform(-10, 10),
                    'rate': 54,
                    'bandwidth': 20,
                    'smoothing': 0,
                    'not_sounding': 0,
                    'aggregation': False,
                    'stbc': 0,
                    'fec_coding': False,
                    'sgi': False,
                    'noise_floor': -95,
                    'amplitude': amplitudes,
                    'phase': phases,
                    'data_length': 200
                }
                
                # Create CSI data object with synthetic data
                csi_data = CSIData(b'\x00' * 200)  # Dummy raw data
                csi_data.parsed_data = demo_data  # Override with our demo data
                
                # Process the demo packet
                await self._process_packet_data(csi_data)
                
                packet_count += 1
                
                # Generate data at ~1 Hz for demo
                await asyncio.sleep(1.0)
                
            except Exception as e:
                logger.error(f"Error in demo loop: {e}")
                await asyncio.sleep(1)
    
    async def _process_packet_data(self, csi_data):
        """Process CSI data (extracted from _process_packet for reuse)"""
        try:
            # Write to file
            if self.data_writer:
                self.data_writer.write_data(csi_data)
            
            # Call data handlers
            for handler in self.data_handlers:
                try:
                    await asyncio.get_event_loop().run_in_executor(None, handler, csi_data)
                except Exception as e:
                    logger.error(f"Error in data handler: {e}")
            
            # Update statistics
            self.stats['packets_received'] += 1
            self.stats['bytes_received'] += len(getattr(csi_data, 'raw_data', b''))
            self.stats['last_packet_time'] = time.time()
            
        except Exception as e:
            logger.error(f"Failed to process packet data: {e}")
    
    async def _process_packet(self, data: bytes):
        """Process a single CSI packet"""
        try:
            # Create CSI data object
            csi_data = CSIData(data)
            
            await self._process_packet_data(csi_data)
            
        except Exception as e:
            logger.error(f"Failed to process packet: {e}")
    
    async def _stats_loop(self):
        """Update statistics periodically"""
        last_packet_count = 0
        
        while self.running:
            try:
                current_time = time.time()
                elapsed = current_time - self._last_stats_update
                
                if elapsed >= 1.0:  # Update every second
                    packets_in_period = self.stats['packets_received'] - last_packet_count
                    self.stats['packets_per_second'] = packets_in_period / elapsed
                    
                    last_packet_count = self.stats['packets_received']
                    self._last_stats_update = current_time
                
                await asyncio.sleep(1)
                
            except Exception as e:
                logger.error(f"Error in stats loop: {e}")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get current statistics"""
        return self.stats.copy()
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        connected = self.serial_manager.connected if self.serial_manager else False
        return {
            'running': self.running,
            'connected': connected,
            'demo_mode': not connected and self.running,
            'port': self.config.get('serial', {}).get('port', ''),
            'stats': self.get_stats()
        }
    
    def update_config(self, new_config: Dict[str, Any]):
        """Update configuration at runtime"""
        try:
            logger.info("Updating collector configuration")
            
            # Store old config for comparison
            old_serial_config = self.config.get('serial', {})
            old_data_config = self.config.get('data', {})
            
            # Update config
            self.config = new_config
            
            # Check if serial configuration changed
            new_serial_config = self.config.get('serial', {})
            if (old_serial_config.get('port') != new_serial_config.get('port') or
                old_serial_config.get('baudrate') != new_serial_config.get('baudrate')):
                logger.info("Serial configuration changed, restarting connection")
                
                # Restart serial connection if running
                if self.running and self.serial_manager:
                    asyncio.create_task(self._restart_serial_connection())
            
            # Check if data configuration changed
            new_data_config = self.config.get('data', {})
            if (old_data_config.get('output_dir') != new_data_config.get('output_dir') or
                old_data_config.get('format') != new_data_config.get('format')):
                logger.info("Data configuration changed")
                
                # Reinitialize data writer if running
                if self.running and self.data_writer:
                    self.data_writer.close()
                    self.data_writer = DataWriter(new_data_config)
            
            logger.info("Configuration updated successfully")
            
        except Exception as e:
            logger.error(f"Failed to update configuration: {e}")
    
    async def _restart_serial_connection(self):
        """Restart serial connection with new configuration"""
        try:
            if self.serial_manager:
                await self.serial_manager.disconnect()
            
            # Wait a moment before reconnecting
            await asyncio.sleep(1)
            
            # Create new serial manager with updated config
            serial_config = self.config.get('serial', {})
            self.serial_manager = SerialManager(serial_config)
            
            # Try to connect
            await self.serial_manager.connect()
            logger.info("Serial connection restarted successfully")
            
        except Exception as e:
            logger.error(f"Failed to restart serial connection: {e}")