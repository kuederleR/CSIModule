#!/usr/bin/env python3
"""
Simple Web UI for logging CSI data from ESP32 devices
Uses csi-py library for reading CSI data
Supports persistent connection with logging and ROS2 publishing
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
from collections import deque

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

# Global CSI Reader Manager
csi_reader_manager = None
csi_reader_lock = threading.Lock()

# Global state
logging_active = False
logging_thread = None
current_logger = None
current_port = None
current_format = 'json'
ros2_publishing = False
ros2_node = None
ros2_thread = None


class CSILogger:
    def __init__(self, port, format_type='json'):
        self.port = port
        self.format_type = format_type
        self.running = False
        self.file = None
        self.csv_writer = None
        self.packet_count = 0
        self.csi_reader = None
        
    def start(self):
        """Start logging CSI data"""
        self.running = True
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        if self.format_type == 'json':
            filename = LOG_DIR / f'csi_log_{timestamp}.jsonl'
            self.file = open(filename, 'w')
        elif self.format_type == 'csv':
            filename = LOG_DIR / f'csi_log_{timestamp}.csv'
            self.file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.file)
            # Write header
            self.csv_writer.writerow(['timestamp', 'packet_num', 'channel', 'bandwidth', 
                                     'noise_floor', 'rate_index', 'rssi', 'mac',
                                     'amplitude_mean', 'amplitude_std', 'phase_mean', 
                                     'phase_std', 'raw_data'])
        elif self.format_type == 'numpy':
            filename = LOG_DIR / f'csi_log_{timestamp}.npz'
            self.data_arrays = {
                'timestamps': [], 
                'rssi': [], 
                'channel': [],
                'amplitude': [], 
                'phase': [],
                'raw_data': []
            }
            self.filename = filename
        
        self.log_filename = filename
        return str(filename)
    
    def log_packet(self, csi_data):
        """Log a single CSI packet from CSIReader callback
        csi_data is a CSIData object with: amplitude, phase, raw, meta
        """
        if not self.running:
            return
        
        self.packet_count += 1
        timestamp = datetime.now().isoformat()
        
        try:
            if self.format_type == 'json':
                # Create JSON record
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
                # Flatten CSI data for CSV
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
                # Accumulate data for numpy
                self.data_arrays['timestamps'].append(timestamp)
                self.data_arrays['rssi'].append(float(csi_data.meta.get('rssi', 0)))
                self.data_arrays['channel'].append(int(csi_data.meta.get('channel', 0)))
                self.data_arrays['amplitude'].append(csi_data.amplitude.copy())
                self.data_arrays['phase'].append(csi_data.phase.copy())
                self.data_arrays['raw_data'].append(list(csi_data.raw))
        except Exception as e:
            print(f"Error logging packet #{self.packet_count} ({self.format_type}): {e}")
            import traceback
            traceback.print_exc()
    
    def stop(self):
        """Stop logging and close file"""
        self.running = False
        
        if self.csi_reader:
            self.csi_reader.stop()
        
        try:
            if self.format_type == 'numpy' and self.packet_count > 0:
                # Save numpy arrays
                print(f"Saving {self.packet_count} packets to numpy file...")
                np.savez_compressed(
                    self.filename,
                    timestamps=np.array(self.data_arrays['timestamps']),
                    rssi=np.array(self.data_arrays['rssi']),
                    channel=np.array(self.data_arrays['channel']),
                    amplitude=np.array(self.data_arrays['amplitude'], dtype=object),
                    phase=np.array(self.data_arrays['phase'], dtype=object),
                    raw_data=np.array(self.data_arrays['raw_data'], dtype=object)
                )
                print(f"NumPy file saved: {self.filename}")
            elif self.file:
                self.file.close()
                print(f"{self.format_type.upper()} file closed: {self.log_filename}")
        except Exception as e:
            print(f"Error closing/saving file: {e}")
            import traceback
            traceback.print_exc()
            
        return self.packet_count


def force_release_serial_port(port):
    """Force release a serial port using system commands"""
    try:
        # Try to find and kill processes using the port
        result = subprocess.run(
            ['fuser', port], 
            capture_output=True, 
            text=True, 
            timeout=2
        )
        if result.stdout.strip():
            print(f"Found processes using {port}: {result.stdout.strip()}")
            # Don't actually kill - just report
    except Exception as e:
        pass  # fuser might not be available


def logging_worker(port, format_type):
    """Background thread for logging CSI data using CSIReader"""
    global logging_active, data_buffer
    
    logger = CSILogger(port, format_type)
    filename = logger.start()
    ser = None
    
    def csi_callback(csi_data):
        """Callback function for CSIReader"""
        if not logging_active:
            return
        
        # Log the packet
        logger.log_packet(csi_data)
        
        # Add to buffer for live display
        data_buffer.append({
            'packet_num': logger.packet_count,
            'timestamp': datetime.now().isoformat(),
            'rssi': csi_data.meta.get('rssi', 'N/A'),
            'channel': csi_data.meta.get('channel', 'N/A'),
            'amplitude_mean': f"{np.mean(csi_data.amplitude):.2f}"
        })
        
        # Keep buffer size limited
        if len(data_buffer) > max_buffer_size:
            data_buffer.pop(0)
    
    # Wait a bit before starting
    time.sleep(0.5)
    force_release_serial_port(port)
    
    try:
        print(f"Starting CSI reader on port {port} with format {format_type}")
        
        # Open serial port ourselves with proper error handling
        ser = serial.Serial(port=port, baudrate=921600, bytesize=8, parity='N', stopbits=1, timeout=1)
        if ser.is_open:
            print(f"Serial port {port} opened successfully")
        else:
            print(f"Failed to open serial port {port}")
            return
        
        # Manual CSI data reading loop (instead of using CSIReader.run())
        last_process = time.time()
        from io import StringIO
        
        while logging_active:
            try:
                line = ser.readline()
                if not line:
                    continue
                    
                strings = line.decode('utf-8', errors='ignore').strip()
                if 'CSI_DATA' not in strings:
                    continue
                
                # Parse CSV data
                csv_reader = csv.reader(StringIO(strings))
                csi_data_raw = next(csv_reader)
                
                # Validate data length
                if len(csi_data_raw) < 15:
                    continue
                
                # Parse based on format (ESP32-C5/C6 or other)
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
                    continue
                
                # Create complex CSI data
                csi_complex = []
                for i in range(len(raw_data) // 2):
                    csi_complex.append(complex(raw_data[i * 2 + 1], raw_data[i * 2]))
                
                csi_array = np.array(csi_complex)
                amplitude = np.abs(csi_array)
                phase = np.angle(csi_array)
                
                # Create CSIData object
                csi_data = CSIData(amplitude, phase, raw_data, meta)
                
                # Call the callback
                csi_callback(csi_data)
                
            except (json.JSONDecodeError, ValueError, IndexError, StopIteration) as e:
                # Skip malformed packets
                continue
            except Exception as e:
                if logging_active:  # Only log if we're still supposed to be running
                    print(f"Error processing packet: {e}")
                break
        
    except serial.SerialException as e:
        print(f"Serial error in logging worker: {e}")
        import traceback
        traceback.print_exc()
    except Exception as e:
        print(f"Error in logging worker: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure proper cleanup
        logging_active = False
        
        # Close serial port
        if ser and ser.is_open:
            try:
                ser.close()
                print(f"Serial port {port} closed")
            except Exception as e:
                print(f"Error closing serial port: {e}")
        
        # Close log files
        packet_count = logger.stop()
        print(f"Logging stopped. Logged {packet_count} packets to {filename}")
        
        # Extra wait to ensure port is fully released
        time.sleep(0.5)


@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')


@app.route('/api/ports')
def get_ports():
    """Get available serial ports"""
    try:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return jsonify({'ports': ports})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/start', methods=['POST'])
def start_logging():
    """Start CSI logging"""
    global logging_active, logging_thread, current_port, current_format, data_buffer
    
    if logging_active:
        return jsonify({'error': 'Logging already active'}), 400
    
    data = request.get_json()
    port = data.get('port')
    format_type = data.get('format', 'json')
    
    if not port:
        return jsonify({'error': 'Port is required'}), 400
    
    current_port = port
    current_format = format_type
    data_buffer = []
    logging_active = True
    
    logging_thread = threading.Thread(target=logging_worker, args=(port, format_type))
    logging_thread.start()
    
    return jsonify({'status': 'started', 'port': port, 'format': format_type})


@app.route('/api/stop', methods=['POST'])
def stop_logging():
    """Stop CSI logging"""
    global logging_active, logging_thread
    
    if not logging_active:
        return jsonify({'error': 'Logging not active'}), 400
    
    logging_active = False
    
    if logging_thread:
        logging_thread.join(timeout=5)
    
    return jsonify({'status': 'stopped'})


@app.route('/api/status')
def get_status():
    """Get current logging status"""
    return jsonify({
        'active': logging_active,
        'port': current_port,
        'format': current_format,
        'buffer': data_buffer[-10:] if data_buffer else []  # Last 10 packets
    })


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
