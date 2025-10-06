#!/usr/bin/env python3
"""
WiFi CSI Data Collector
Main application entry point
"""

import argparse
import asyncio
import logging
import os
import signal
import sys
from pathlib import Path

from src.collector import CSICollector
from src.api_server import APIServer
from src.config_manager import ConfigManager
from src.ros2_bridge import ROS2Bridge

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/app/logs/csi_collector.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

logger = logging.getLogger(__name__)


class CSICollectorApp:
    """Main application class for CSI data collection"""
    
    def __init__(self, config_path: str, enable_ros2: bool = False):
        self.config_path = config_path
        self.enable_ros2 = enable_ros2
        self.config_manager = None
        self.collector = None
        self.api_server = None
        self.ros2_bridge = None
        self.running = False
        
    async def initialize(self):
        """Initialize all components"""
        try:
            # Load configuration
            self.config_manager = ConfigManager(self.config_path)
            config = self.config_manager.get_config()
            
            logger.info("Starting CSI Collector Application")
            logger.info(f"Configuration loaded from: {self.config_path}")
            
            # Initialize CSI collector
            self.collector = CSICollector(config)
            
            # Initialize API server
            self.api_server = APIServer(self.collector, config, self.config_manager)
            
            # Initialize ROS2 bridge if enabled
            if self.enable_ros2 and config.get('ros2', {}).get('enabled', False):
                try:
                    self.ros2_bridge = ROS2Bridge(config)
                    # Connect collector to ROS2 bridge
                    self.collector.add_data_handler(self.ros2_bridge.publish_csi_data)
                    logger.info("ROS2 bridge initialized")
                except Exception as e:
                    logger.error(f"Failed to initialize ROS2 bridge: {e}")
                    self.ros2_bridge = None
            
            # Connect collector to API server for real-time updates
            self.collector.add_data_handler(self.api_server.broadcast_data)
            
            logger.info("All components initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize application: {e}")
            raise
    
    async def start(self):
        """Start all services"""
        try:
            self.running = True
            
            # Start collector
            await self.collector.start()
            
            # Start ROS2 bridge if available
            if self.ros2_bridge:
                await self.ros2_bridge.start()
            
            # Start API server (blocking)
            await self.api_server.start()
            
        except Exception as e:
            logger.error(f"Failed to start application: {e}")
            await self.shutdown()
            raise
    
    async def shutdown(self):
        """Gracefully shutdown all components"""
        if not self.running:
            return
            
        logger.info("Shutting down CSI Collector Application")
        self.running = False
        
        # Stop collector
        if self.collector:
            await self.collector.stop()
        
        # Stop ROS2 bridge
        if self.ros2_bridge:
            await self.ros2_bridge.stop()
        
        # Stop API server
        if self.api_server:
            await self.api_server.stop()
        
        logger.info("Application shutdown complete")


def signal_handler(app):
    """Handle shutdown signals"""
    def handler(signum, frame):
        logger.info(f"Received signal {signum}, starting shutdown...")
        asyncio.create_task(app.shutdown())
    return handler


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='WiFi CSI Data Collector')
    parser.add_argument(
        '--config', 
        default=os.getenv('CONFIG_PATH', '/app/config/config.yaml'),
        help='Path to configuration file'
    )
    parser.add_argument(
        '--ros2', 
        action='store_true',
        help='Enable ROS2 integration'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug logging'
    )
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Ensure directories exist
    os.makedirs('/app/data', exist_ok=True)
    os.makedirs('/app/logs', exist_ok=True)
    os.makedirs('/app/config', exist_ok=True)
    
    # Create default config if it doesn't exist
    if not os.path.exists(args.config):
        logger.warning(f"Config file not found at {args.config}, creating default")
        ConfigManager.create_default_config(args.config)
    
    # Initialize application
    app = CSICollectorApp(args.config, args.ros2)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler(app))
    signal.signal(signal.SIGTERM, signal_handler(app))
    
    try:
        await app.initialize()
        await app.start()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Application error: {e}")
        sys.exit(1)
    finally:
        await app.shutdown()


if __name__ == '__main__':
    asyncio.run(main())