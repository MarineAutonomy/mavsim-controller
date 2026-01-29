"""
Command-Line Interface for MAVSIM Sensor Bridge

Provides a standalone CLI for running the sensor bridge with configurable options.
"""

import argparse
import asyncio
import logging
import signal
import sys
from typing import Optional

from mavsim_sensor_bridge.bridge import BridgeConfig, SensorBridge


def parse_args(args: Optional[list] = None) -> argparse.Namespace:
    """
    Parse command-line arguments.
    
    Args:
        args: Optional list of arguments to parse. If None, uses sys.argv.
    
    Returns:
        Parsed arguments namespace
    """
    parser = argparse.ArgumentParser(
        description="MAVSIM Local Sensor Bridge - High-bandwidth perception sensor streaming",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --verbose --stats-interval 5
  %(prog)s --camera-port 9000
  %(prog)s --camera-port 8765 --stats-interval 10
        """
    )
    
    parser.add_argument(
        '--camera-port',
        type=int,
        default=8765,
        help='Port for camera server (default: 8765)'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable debug logging'
    )
    
    parser.add_argument(
        '--stats-interval',
        type=float,
        default=0,
        help='Interval in seconds for periodic statistics reporting (0 = disabled, default: 0)'
    )
    
    if args is None:
        return parser.parse_args()
    else:
        return parser.parse_args(args)


class CLIBridgeRunner:
    """
    CLI runner for SensorBridge with signal handling and statistics reporting.
    
    Handles graceful shutdown on SIGINT/SIGTERM and manages periodic statistics
    reporting if enabled.
    """
    
    def __init__(self, args: argparse.Namespace):
        """
        Initialize CLI bridge runner.
        
        Args:
            args: Parsed command-line arguments
        """
        self.args = args
        self.bridge: Optional[SensorBridge] = None
        self.stats_task: Optional[asyncio.Task] = None
        self.shutdown_event = asyncio.Event()
        self.logger = logging.getLogger(f"{__name__}.CLIBridgeRunner")
        
        # Set up logging level
        log_level = logging.DEBUG if args.verbose else logging.INFO
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """
        Handle SIGINT/SIGTERM signals for graceful shutdown.
        
        Args:
            signum: Signal number
            frame: Current stack frame
        """
        try:
            signal_name = signal.Signals(signum).name
        except (ValueError, AttributeError):
            signal_name = f"signal-{signum}"
        self.logger.info(f"Received {signal_name}, initiating graceful shutdown...")
        self.shutdown_event.set()
    
    async def _periodic_stats_loop(self) -> None:
        """
        Background task that periodically logs bridge statistics.
        
        This runs in an asyncio task and logs stats at the configured interval.
        """
        while not self.shutdown_event.is_set():
            try:
                await asyncio.sleep(self.args.stats_interval)
                
                if self.bridge and self.bridge.is_running:
                    stats = self.bridge.get_server_stats()
                    self.logger.info("=" * 60)
                    self.logger.info("Bridge Statistics Report")
                    self.logger.info("=" * 60)
                    for sensor_type, sensor_stats in stats.items():
                        self.logger.info(
                            f"{sensor_type}: "
                            f"{sensor_stats.get('messages', 0)} msgs, "
                            f"{sensor_stats.get('bytes', 0):,} bytes, "
                            f"{sensor_stats.get('connections', 0)} connections"
                        )
                    self.logger.info("=" * 60)
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Error in periodic stats loop: {e}")
    
    async def run(self) -> int:
        """
        Run the bridge with CLI configuration.
        
        Returns:
            Exit code (0 for success, 1 for error)
        """
        try:
            # Create bridge configuration
            config = BridgeConfig(
                camera_port=self.args.camera_port,
                camera_enabled=True,
                log_level=logging.DEBUG if self.args.verbose else logging.INFO
            )
            
            # Create bridge instance
            self.bridge = SensorBridge(config=config)
            self.logger.info(f"Initialized SensorBridge with camera_port={self.args.camera_port}")
            
            # Set up periodic statistics reporting if interval is specified
            if self.args.stats_interval > 0:
                self.stats_task = asyncio.create_task(self._periodic_stats_loop())
                self.logger.info(f"Statistics reporting enabled (interval: {self.args.stats_interval}s)")
            
            # Start bridge in background task
            bridge_task = asyncio.create_task(self.bridge.start())
            
            # Wait for shutdown signal or bridge completion
            self.logger.info("SensorBridge started. Press Ctrl+C to stop.")
            
            # Wait for shutdown event or bridge task completion
            done, pending = await asyncio.wait(
                [asyncio.create_task(self.shutdown_event.wait()), bridge_task],
                return_when=asyncio.FIRST_COMPLETED
            )
            
            # If shutdown was requested, stop the bridge gracefully
            if self.shutdown_event.is_set():
                self.logger.info("Stopping SensorBridge...")
                await self.bridge.stop()
                
                # Cancel bridge task if still running
                if not bridge_task.done():
                    bridge_task.cancel()
                    try:
                        await bridge_task
                    except asyncio.CancelledError:
                        pass
            
            # Stop statistics reporting task if it was started
            if self.stats_task and not self.stats_task.done():
                self.stats_task.cancel()
                try:
                    await self.stats_task
                except asyncio.CancelledError:
                    pass
            
            self.logger.info("SensorBridge stopped successfully")
            return 0
            
        except KeyboardInterrupt:
            self.logger.info("Interrupted by user")
            if self.bridge:
                await self.bridge.stop()
            return 0
        except Exception as e:
            self.logger.error(f"Error running bridge: {e}", exc_info=True)
            if self.bridge:
                try:
                    await self.bridge.stop()
                except Exception:
                    pass
            return 1


def main() -> int:
    """
    Main entry point for the CLI.
    
    Returns:
        Exit code (0 for success, 1 for error)
    """
    args = parse_args()
    
    runner = CLIBridgeRunner(args)
    
    try:
        return asyncio.run(runner.run())
    except KeyboardInterrupt:
        return 0
    except Exception as e:
        logging.error(f"Fatal error: {e}", exc_info=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
