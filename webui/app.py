#!/usr/bin/env python3
"""
Simple Web UI for logging CSI data from ESP32 devices
Uses persistent CSI reader with queue-based architecture
Supports logging to file and publishing to ROS2
"""

from flask import Flask, render_template, request, jsonify, send_file
import threading
import json
import csv
import os
from datetime import datetime
from pathlib import Path
import numpy as np
import serial
import serial.tools.list_ports
import time
import queue
from io import StringIO

# Import csi-py for CSIData type
try:
    from csi_py.csireader import CSIData
except ImportError:
    print("Error: csi-py not installed. Install with: pip install csi-py")
    exit(1)

# Try to import ROS2
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from wifi_msgs.msg import CSI as CSIMsg
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
    print("ROS2 support enabled")
except ImportError:
    print("ROS2 not available. Publishing to ROS2 will be disabled.")

app = Flask(__name__)

# Directory for log files
LOG_DIR = Path('/app/logs')
LOG_DIR.mkdir(exist_ok=True)

# Global state
csi_reader_manager = None
current_logger = None
ros2_publisher = None
data_buffer = []
MAX_BUFFER_SIZE = 100


class CSIReaderManager:
    """Persistent CSI reader that feeds data to multiple consumers"""
    
    def __init__(self, port):
        self.port = port
        self.running = False
        self.ser = None
        self.thread = None
        self.consumers = []  # List of queues for data consumers
        self.lock = threading.Lock()
        self.packet_count = 0
        
    def add_consumer(self):
        """Add a consumer queue"""
        q = queue.Queue(maxsize=1000)
        with self.lock:
            self.consumers.append(q)
        return q
    
    def remove_consumer(self, q):
        """Remove a consumer queue"""
        with self.lock:
            if q in self.consumers:
                self.consumers.remove(q)
    
    def start(self):
        """Start the persistent CSI reader"""
        if self.running:
            return True
        
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=921600,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            
            if not self.ser.is_open:
                print(f"Failed to open port {self.port}")
                return False
            
            print(f"CSI Reader: Port {self.port} opened successfully")
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
            
        except Exception as e:
            print(f"Error starting CSI reader: {e}")
            return False
    
    def stop(self):
        """Stop the CSI reader"""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2)
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print(f"CSI Reader: Port {self.port} closed")
            except Exception as e:
                print(f"Error closing port: {e}")
        
        # Clear all consumer queues
        with self.lock:
            for q in self.consumers:
                while not q.empty():
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        break
            self.consumers.clear()
    
    def _read_loop(self):
        """Main reading loop"""
        while self.running:
            try:
                line = self.ser.readline()
                if not line:
                    continue
                
                strings = line.decode('utf-8', errors='ignore').strip()
                if 'CSI_DATA' not in strings:
                    continue
                
                # Parse CSI data
                csi_data = self._parse_csi_data(strings)
                if csi_data:
                    self.packet_count += 1
                    
                    # Distribute to all consumers
                    with self.lock:
                        for q in self.consumers:
                            try:
                                q.put_nowait(csi_data)
                            except queue.Full:
                                # Consumer is slow, skip this packet for them
                                pass
                    
                    # Update live buffer
                    global data_buffer
                    data_buffer.append({
                        'packet_num': self.packet_count,
                        'timestamp': datetime.now().isoformat(),
                        'rssi': csi_data.meta.get('rssi', 'N/A'),
                        'channel': csi_data.meta.get('channel', 'N/A'),
                        'amplitude_mean': f"{np.mean(csi_data.amplitude):.2f}"
                    })
                    if len(data_buffer) > MAX_BUFFER_SIZE:
                        data_buffer.pop(0)
                        
            except Exception as e:
                if self.running:
                    print(f"Error in CSI read loop: {e}")
                    time.sleep(0.1)
    
    def _parse_csi_data(self, strings):
        """Parse CSI data from string"""
        try:
            csv_reader = csv.reader(StringIO(strings))
            csi_data_raw = next(csv_reader)
            
            if len(csi_data_raw) < 15:
                return None
            
            # Parse based on format
            if len(csi_data_raw) == 15:  # C5/C6 format
                meta = {
                    "channel": csi_data_raw[8],
                    "bandwidth": None,
                    "noise_floor": csi_data_raw[5],
                    "rate_index": csi_data_raw[4],
                    "rssi": csi_data_raw[3],
                    "mac": csi_data_raw[2]
                }
                raw_data = json.loads(csi_data_raw[-1])
            elif len(csi_data_raw) == 25:  # Standard format
                meta = {
                    "channel": csi_data_raw[16],
                    "bandwidth": csi_data_raw[7],
                    "noise_floor": csi_data_raw[14],
                    "rate_index": csi_data_raw[6],
                    "rssi": csi_data_raw[3],
                    "mac": csi_data_raw[2]
                }
                raw_data = json.loads(csi_data_raw[-1])
            else:
                return None
            
            # Create complex CSI data
            csi_complex = []
            for i in range(len(raw_data) // 2):
                csi_complex.append(complex(raw_data[i * 2 + 1], raw_data[i * 2]))
            
            csi_array = np.array(csi_complex)
            amplitude = np.abs(csi_array)
            phase = np.angle(csi_array)
            
            return CSIData(amplitude, phase, raw_data, meta)
            
        except (json.JSONDecodeError, ValueError, IndexError, StopIteration):
            return None


class CSILogger:
    """Logger that consumes CSI data from the reader manager"""
    
    def __init__(self, reader_manager, format_type='json'):
        self.reader_manager = reader_manager
        self.format_type = format_type
        self.running = False
        self.file = None
        self.csv_writer = None
        self.packet_count = 0
        self.consumer_queue = None
        self.thread = None
        self.data_arrays = None
        
    def start(self):
        """Start logging"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        if self.format_type == 'json':
            filename = LOG_DIR / f'csi_log_{timestamp}.jsonl'
            self.file = open(filename, 'w')
        elif self.format_type == 'csv':
            filename = LOG_DIR / f'csi_log_{timestamp}.csv'
            self.file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.file)
            self.csv_writer.writerow([
                'timestamp', 'packet_num', 'channel', 'bandwidth',
                'noise_floor', 'rate_index', 'rssi', 'mac',
                'amplitude_mean', 'amplitude_std', 'phase_mean',
                'phase_std', 'raw_data'
            ])
        elif self.format_type == 'numpy':
            filename = LOG_DIR / f'csi_log_{timestamp}.npz'
            self.filename = filename
            self.data_arrays = {
                'timestamps': [],
                'rssi': [],
                'channel': [],
                'amplitude': [],
                'phase': [],
                'raw_data': []
            }
        
        self.log_filename = filename
        self.running = True
        self.consumer_queue = self.reader_manager.add_consumer()
        self.thread = threading.Thread(target=self._log_loop, daemon=True)
        self.thread.start()
        
        return str(filename)
    
    def _log_loop(self):
        """Logging loop that consumes from queue"""
        while self.running:
            try:
                csi_data = self.consumer_queue.get(timeout=1)
                self._log_packet(csi_data)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in logging loop: {e}")
                import traceback
                traceback.print_exc()
    
    def _log_packet(self, csi_data):
        """Log a single CSI packet"""
        self.packet_count += 1
        timestamp = datetime.now().isoformat()
        
        try:
            if self.format_type == 'json':
                record = {
                    'timestamp': timestamp,
                    'packet_num': self.packet_count,
                    'meta': csi_data.meta,
                    'amplitude': csi_data.amplitude.tolist(),
                    'phase': csi_data.phase.tolist(),
                    'raw': csi_data.raw
                }
                self.file.write(json.dumps(record) + '\n')
                self.file.flush()
                
            elif self.format_type == 'csv':
                row = [
                    timestamp,
                    self.packet_count,
                    csi_data.meta.get('channel', ''),
                    csi_data.meta.get('bandwidth', ''),
                    csi_data.meta.get('noise_floor', ''),
                    csi_data.meta.get('rate_index', ''),
                    csi_data.meta.get('rssi', ''),
                    csi_data.meta.get('mac', ''),
                    float(np.mean(csi_data.amplitude)),
                    float(np.std(csi_data.amplitude)),
                    float(np.mean(csi_data.phase)),
                    float(np.std(csi_data.phase)),
                    json.dumps(csi_data.raw)
                ]
                self.csv_writer.writerow(row)
                self.file.flush()
                
            elif self.format_type == 'numpy':
                self.data_arrays['timestamps'].append(timestamp)
                self.data_arrays['rssi'].append(float(csi_data.meta.get('rssi', 0)))
                self.data_arrays['channel'].append(int(csi_data.meta.get('channel', 0)))
                self.data_arrays['amplitude'].append(csi_data.amplitude.copy())
                self.data_arrays['phase'].append(csi_data.phase.copy())
                self.data_arrays['raw_data'].append(list(csi_data.raw))
                
        except Exception as e:
            print(f"Error logging packet: {e}")
            import traceback
            traceback.print_exc()
    
    def stop(self):
        """Stop logging"""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2)
        
        if self.consumer_queue:
            self.reader_manager.remove_consumer(self.consumer_queue)
        
        try:
            if self.format_type == 'numpy' and self.packet_count > 0:
                np.savez_compressed(
                    self.filename,
                    timestamps=np.array(self.data_arrays['timestamps']),
                    rssi=np.array(self.data_arrays['rssi']),
                    channel=np.array(self.data_arrays['channel']),
                    amplitude=np.array(self.data_arrays['amplitude'], dtype=object),
                    phase=np.array(self.data_arrays['phase'], dtype=object),
                    raw_data=np.array(self.data_arrays['raw_data'], dtype=object)
                )
            elif self.file:
                self.file.close()
        except Exception as e:
            print(f"Error closing log file: {e}")
        
        print(f"Logged {self.packet_count} packets to {self.log_filename}")
        return self.packet_count


class ROS2CSIPublisher:
    """ROS2 publisher for CSI data - Consumer-based"""
    
    def __init__(self, reader_manager):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 is not available")
        
        self.reader_manager = reader_manager
        self.running = False
        self.consumer_queue = None
        self.thread = None
        self.node = None
        self.publisher = None
        self.packet_count = 0
    
    def start(self):
        """Start ROS2 publishing"""
        if not ROS2_AVAILABLE:
            return False
        
        try:
            # Initialize ROS2 if not already initialized
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('csi_web_publisher')
            self.publisher = self.node.create_publisher(CSIMsg, 'csi_data', 10)
            
            self.running = True
            self.consumer_queue = self.reader_manager.add_consumer()
            self.thread = threading.Thread(target=self._publish_loop, daemon=True)
            self.thread.start()
            
            print("ROS2 CSI Publisher started on topic: csi_data")
            return True
            
        except Exception as e:
            print(f"Error starting ROS2 publisher: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _publish_loop(self):
        """Publishing loop - consumes from queue"""
        while self.running:
            try:
                csi_data = self.consumer_queue.get(timeout=1)
                self._publish_csi(csi_data)
                # Spin once to process callbacks
                rclpy.spin_once(self.node, timeout_sec=0)
            except queue.Empty:
                # Keep ROS2 node alive
                if self.node:
                    rclpy.spin_once(self.node, timeout_sec=0)
            except Exception as e:
                if self.running:
                    print(f"Error in ROS2 publish loop: {e}")
                    import traceback
                    traceback.print_exc()
    
    def _publish_csi(self, csi_data):
        """Publish a single CSI message - matching proper CSI publisher pattern"""
        try:
            msg = CSIMsg()
            
            # Header with timestamp from ROS2 clock
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "csi_frame"
            
            # Basic WiFi information - convert types properly like reference implementation
            msg.mac = str(csi_data.meta.get('mac', ''))
            msg.rssi = int(csi_data.meta.get('rssi', 0))
            msg.channel = int(csi_data.meta.get('channel', 0))
            
            # CSI Matrix dimensions
            msg.num_subcarriers = len(csi_data.amplitude)
            
            # CSI Data - convert to float lists like reference implementation
            msg.csi_complex = [float(x) for x in csi_data.raw]
            msg.csi_amplitude = [float(x) for x in csi_data.amplitude]
            msg.csi_phase = [float(x) for x in csi_data.phase]
            
            self.publisher.publish(msg)
            self.packet_count += 1
            
        except Exception as e:
            print(f"Error publishing CSI message: {e}")
            import traceback
            traceback.print_exc()
    
    def stop(self):
        """Stop ROS2 publishing"""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=2)
        
        if self.consumer_queue:
            self.reader_manager.remove_consumer(self.consumer_queue)
        
        if self.node:
            self.node.destroy_node()
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print(f"ROS2 Publisher stopped. Published {self.packet_count} messages")


# Flask Routes

@app.route('/')
def index():
    """Main page"""
    return render_template('index.html', ros2_available=ROS2_AVAILABLE)


@app.route('/api/ports')
def get_ports():
    """Get available serial ports"""
    try:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return jsonify({'ports': ports})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/connect', methods=['POST'])
def connect_port():
    """Connect to a serial port"""
    global csi_reader_manager
    
    data = request.get_json()
    port = data.get('port')
    
    if not port:
        return jsonify({'error': 'Port is required'}), 400
    
    # Stop existing connection
    if csi_reader_manager:
        csi_reader_manager.stop()
        time.sleep(0.5)
    
    # Create new reader manager
    csi_reader_manager = CSIReaderManager(port)
    if csi_reader_manager.start():
        return jsonify({'status': 'connected', 'port': port})
    else:
        csi_reader_manager = None
        return jsonify({'error': 'Failed to connect to port'}), 500


@app.route('/api/disconnect', methods=['POST'])
def disconnect_port():
    """Disconnect from serial port"""
    global csi_reader_manager, current_logger, ros2_publisher
    
    # Stop logging
    if current_logger:
        current_logger.stop()
        current_logger = None
    
    # Stop ROS2
    if ros2_publisher:
        ros2_publisher.stop()
        ros2_publisher = None
    
    # Stop reader
    if csi_reader_manager:
        csi_reader_manager.stop()
        csi_reader_manager = None
    
    return jsonify({'status': 'disconnected'})


@app.route('/api/logging/start', methods=['POST'])
def start_logging():
    """Start logging CSI data"""
    global current_logger
    
    if not csi_reader_manager or not csi_reader_manager.running:
        return jsonify({'error': 'Not connected to a port'}), 400
    
    if current_logger and current_logger.running:
        return jsonify({'error': 'Logging already active'}), 400
    
    data = request.get_json()
    format_type = data.get('format', 'json')
    
    current_logger = CSILogger(csi_reader_manager, format_type)
    filename = current_logger.start()
    
    return jsonify({'status': 'started', 'format': format_type, 'filename': filename})


@app.route('/api/logging/stop', methods=['POST'])
def stop_logging():
    """Stop logging CSI data"""
    global current_logger
    
    if not current_logger or not current_logger.running:
        return jsonify({'error': 'Logging not active'}), 400
    
    packet_count = current_logger.stop()
    current_logger = None
    
    return jsonify({'status': 'stopped', 'packet_count': packet_count})


@app.route('/api/ros2/start', methods=['POST'])
def start_ros2():
    """Start ROS2 publishing"""
    global ros2_publisher
    
    if not ROS2_AVAILABLE:
        return jsonify({'error': 'ROS2 is not available'}), 400
    
    if not csi_reader_manager or not csi_reader_manager.running:
        return jsonify({'error': 'Not connected to a port'}), 400
    
    if ros2_publisher and ros2_publisher.running:
        return jsonify({'error': 'ROS2 publishing already active'}), 400
    
    ros2_publisher = ROS2CSIPublisher(csi_reader_manager)
    if ros2_publisher.start():
        return jsonify({'status': 'started'})
    else:
        ros2_publisher = None
        return jsonify({'error': 'Failed to start ROS2 publishing'}), 500


@app.route('/api/ros2/stop', methods=['POST'])
def stop_ros2():
    """Stop ROS2 publishing"""
    global ros2_publisher
    
    if not ros2_publisher or not ros2_publisher.running:
        return jsonify({'error': 'ROS2 publishing not active'}), 400
    
    ros2_publisher.stop()
    ros2_publisher = None
    
    return jsonify({'status': 'stopped'})


@app.route('/api/status')
def get_status():
    """Get current status"""
    status = {
        'connected': csi_reader_manager is not None and csi_reader_manager.running,
        'port': csi_reader_manager.port if csi_reader_manager else None,
        'packet_count': csi_reader_manager.packet_count if csi_reader_manager else 0,
        'logging_active': current_logger is not None and current_logger.running,
        'logging_format': current_logger.format_type if current_logger else None,
        'logging_count': current_logger.packet_count if current_logger else 0,
        'ros2_active': ros2_publisher is not None and ros2_publisher.running,
        'ros2_count': ros2_publisher.packet_count if ros2_publisher else 0,
        'ros2_available': ROS2_AVAILABLE,
        'buffer': data_buffer[-10:] if data_buffer else []
    }
    return jsonify(status)


@app.route('/api/logs')
def list_logs():
    """List available log files"""
    try:
        files = []
        for file in LOG_DIR.iterdir():
            if file.is_file():
                files.append({
                    'name': file.name,
                    'size': file.stat().st_size,
                    'modified': datetime.fromtimestamp(file.stat().st_mtime).isoformat()
                })
        files.sort(key=lambda x: x['modified'], reverse=True)
        return jsonify({'files': files})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/download/<filename>')
def download_log(filename):
    """Download a log file"""
    try:
        file_path = LOG_DIR / filename
        if not file_path.exists() or not file_path.is_file():
            return jsonify({'error': 'File not found'}), 404
        
        return send_file(file_path, as_attachment=True)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/delete/<filename>', methods=['DELETE'])
def delete_log(filename):
    """Delete a log file"""
    try:
        file_path = LOG_DIR / filename
        if not file_path.exists() or not file_path.is_file():
            return jsonify({'error': 'File not found'}), 404
        
        file_path.unlink()
        return jsonify({'status': 'deleted'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
