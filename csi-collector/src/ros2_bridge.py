"""
ROS2 Bridge for CSI Collector
Publishes CSI data to ROS2 topics
"""

import asyncio
import json
import logging
import time
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Header, String
    from sensor_msgs.msg import PointCloud2, PointField
    from geometry_msgs.msg import Vector3
    import threading
    ROS2_AVAILABLE = True
except ImportError:
    logger.warning("ROS2 not available. ROS2 bridge will be disabled.")
    ROS2_AVAILABLE = False
    # Create dummy classes to prevent import errors
    class Node:
        pass
    class QoSProfile:
        pass
    class Header:
        pass
    class PointCloud2:
        pass
    class PointField:
        pass


class CSIMessage:
    """Custom message type for CSI data"""
    
    def __init__(self):
        self.header = None
        self.channel = 0
        self.rssi = 0
        self.bandwidth = 0
        self.num_subcarriers = 0
        self.amplitude = []
        self.phase = []
        self.timestamp = 0.0
        self.device_id = ""


class CSIPublisher(Node):
    """ROS2 node for publishing CSI data"""
    
    def __init__(self, config: Dict[str, Any]):
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available")
        
        super().__init__('csi_publisher')
        
        self.config = config
        ros2_config = config.get('ros2', {})
        
        # Setup QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        topic_name = ros2_config.get('topic', '/csi_data')
        
        # Publish as JSON string for simplicity
        self.csi_publisher = self.create_publisher(
            String, 
            topic_name, 
            qos_profile
        )
        
        # Publish as PointCloud2 for visualization
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            f"{topic_name}/pointcloud",
            qos_profile
        )
        
        # Statistics
        self.stats = {
            'messages_published': 0,
            'last_publish_time': None,
            'publish_rate': 0.0
        }
        
        self.get_logger().info(f"CSI Publisher initialized on topic: {topic_name}")
    
    def publish_csi_data(self, csi_data):
        """Publish CSI data to ROS2 topics"""
        try:
            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "csi_sensor"
            
            # Publish as JSON string
            json_msg = String()
            json_msg.data = json.dumps(csi_data.to_dict())
            self.csi_publisher.publish(json_msg)
            
            # Publish as PointCloud2 for visualization
            pointcloud_msg = self._create_pointcloud(csi_data, header)
            if pointcloud_msg:
                self.pointcloud_publisher.publish(pointcloud_msg)
            
            # Update statistics
            self.stats['messages_published'] += 1
            current_time = time.time()
            if self.stats['last_publish_time']:
                elapsed = current_time - self.stats['last_publish_time']
                if elapsed > 0:
                    self.stats['publish_rate'] = 1.0 / elapsed
            self.stats['last_publish_time'] = current_time
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish CSI data: {e}")
    
    def _create_pointcloud(self, csi_data, header):
        """Create PointCloud2 message from CSI data for visualization"""
        try:
            parsed_data = csi_data.parsed_data
            amplitudes = parsed_data.get('amplitude', [])
            phases = parsed_data.get('phase', [])
            
            if not amplitudes or not phases:
                return None
            
            # Create PointCloud2 message
            pointcloud = PointCloud2()
            pointcloud.header = header
            pointcloud.height = 1
            pointcloud.width = len(amplitudes)
            
            # Define fields
            pointcloud.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            pointcloud.is_bigendian = False
            pointcloud.point_step = 16  # 4 fields * 4 bytes each
            pointcloud.row_step = pointcloud.point_step * pointcloud.width
            
            # Create point data
            import struct
            point_data = b''
            for i, (amp, phase) in enumerate(zip(amplitudes, phases)):
                # Use subcarrier index as x, amplitude as y, phase as z
                x = float(i)
                y = float(amp)
                z = float(phase)
                intensity = float(amp)
                
                point_data += struct.pack('<ffff', x, y, z, intensity)
            
            pointcloud.data = point_data
            pointcloud.is_dense = True
            
            return pointcloud
            
        except Exception as e:
            self.get_logger().error(f"Failed to create pointcloud: {e}")
            return None
    
    def get_stats(self) -> Dict[str, Any]:
        """Get publisher statistics"""
        return self.stats.copy()


class ROS2Bridge:
    """Bridge between CSI collector and ROS2"""
    
    def __init__(self, config: Dict[str, Any]):
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 not available")
        
        self.config = config
        self.publisher = None
        self.executor = None
        self.thread = None
        self.running = False
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
    
    async def start(self):
        """Start ROS2 bridge"""
        if self.running:
            return
        
        try:
            # Create publisher node
            self.publisher = CSIPublisher(self.config)
            
            # Create executor and run in separate thread
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.publisher)
            
            self.thread = threading.Thread(target=self._run_executor, daemon=True)
            self.thread.start()
            
            self.running = True
            logger.info("ROS2 bridge started")
            
        except Exception as e:
            logger.error(f"Failed to start ROS2 bridge: {e}")
            raise
    
    async def stop(self):
        """Stop ROS2 bridge"""
        if not self.running:
            return
        
        self.running = False
        
        if self.executor:
            self.executor.shutdown()
        
        if self.publisher:
            self.publisher.destroy_node()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5.0)
        
        logger.info("ROS2 bridge stopped")
    
    def _run_executor(self):
        """Run ROS2 executor in separate thread"""
        try:
            while self.running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            logger.error(f"Error in ROS2 executor: {e}")
    
    def publish_csi_data(self, csi_data):
        """Publish CSI data to ROS2 (called from collector)"""
        if self.running and self.publisher:
            try:
                self.publisher.publish_csi_data(csi_data)
            except Exception as e:
                logger.error(f"Failed to publish to ROS2: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """Get ROS2 bridge status"""
        stats = {}
        if self.publisher:
            stats = self.publisher.get_stats()
        
        return {
            'running': self.running,
            'ros2_available': ROS2_AVAILABLE,
            'connected': self.running and rclpy.ok() if ROS2_AVAILABLE else False,
            'stats': stats
        }