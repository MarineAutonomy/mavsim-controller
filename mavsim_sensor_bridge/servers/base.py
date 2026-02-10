"""
Base Sensor Server Class

Abstract base class for all sensor WebSocket servers with common functionality
including connection tracking, statistics, and lifecycle management.
"""

import asyncio
import logging
from abc import ABC, abstractmethod
from typing import Set, TYPE_CHECKING
import websockets

if TYPE_CHECKING:
    # Type hint only - avoids deprecation warning at runtime
    from websockets.server import WebSocketServerProtocol
else:
    # Use a generic type at runtime to avoid deprecation
    WebSocketServerProtocol = object


class BaseSensorServer(ABC):
    """
    Abstract base class for sensor WebSocket servers.
    
    Provides common functionality for:
    - Async WebSocket server lifecycle (start, stop)
    - Connection tracking
    - Basic statistics (messages received, bytes received)
    - Configurable logging
    
    Subclasses must implement `_process_message()` to handle incoming messages.
    
    Attributes:
        port (int): Port number for the WebSocket server
        name (str): Name identifier for this server instance
        connections (Set[WebSocketServerProtocol]): Set of active WebSocket connections
        stats (dict): Statistics dictionary with 'messages' and 'bytes' counters
        logger (logging.Logger): Logger instance for this server
        _server: Internal WebSocket server instance
        _is_running (bool): Flag indicating if server is currently running
    """
    
    # Maximum incoming WebSocket frame size in bytes.
    # Subclasses may override this (e.g. LidarSensorServer sets a higher
    # limit because a single 100K-point scan is ~1.6 MB).
    # None means no limit.
    _ws_max_size: int | None = None

    def __init__(self, port: int, name: str, log_level: int = logging.INFO):
        """
        Initialize the base sensor server.
        
        Args:
            port: Port number for the WebSocket server
            name: Name identifier for this server instance
            log_level: Logging level (default: logging.INFO)
        """
        self.port = port
        self.name = name
        self.connections: Set[WebSocketServerProtocol] = set()
        self.stats = {'messages': 0, 'bytes': 0}
        self._server = None
        self._is_running = False
        
        # Set up logging
        self.logger = logging.getLogger(f"{__name__}.{name}")
        self.logger.setLevel(log_level)
        
        # Create console handler if no handlers exist
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            handler.setLevel(log_level)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
    
    @property
    def is_running(self) -> bool:
        """Check if the server is currently running."""
        return self._is_running
    
    async def start(self) -> None:
        """
        Start the WebSocket server.
        
        This method is async and will block until the server is stopped.
        It should be run in an asyncio task or event loop.
        
        Raises:
            OSError: If the port is already in use
        """
        if self._is_running:
            self.logger.warning(f"Server {self.name} is already running")
            return
        
        try:
            self.logger.info(f"Starting {self.name} server on port {self.port}")
            self._server = await websockets.serve(
                self._handle_connection,
                None,  # Listen on all interfaces (IPv4 + IPv6) to accept browser & Docker connections
                self.port,
                max_size=self._ws_max_size,
            )
            self._is_running = True
            self.logger.info(f"{self.name} server started on port {self.port}")
            
            # Wait for the server to be closed
            await self._server.wait_closed()
        except OSError as e:
            self.logger.error(f"Failed to start {self.name} server on port {self.port}: {e}")
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error in {self.name} server: {e}")
            raise
    
    async def stop(self) -> None:
        """
        Stop the WebSocket server gracefully.
        
        Closes all active connections and shuts down the server.
        """
        if not self._is_running:
            self.logger.warning(f"Server {self.name} is not running")
            return
        
        self.logger.info(f"Stopping {self.name} server...")
        self._is_running = False
        
        # Close all active connections
        if self.connections:
            self.logger.info(f"Closing {len(self.connections)} active connection(s)")
            # Create a list of close tasks to avoid modifying set during iteration
            close_tasks = [
                ws.close() for ws in list(self.connections)
            ]
            await asyncio.gather(*close_tasks, return_exceptions=True)
            self.connections.clear()
        
        # Close the server
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        
        self.logger.info(f"{self.name} server stopped")
    
    async def _handle_connection(self, websocket: WebSocketServerProtocol) -> None:
        """
        Handle a new WebSocket connection.
        
        This method is called by websockets.serve() for each new connection.
        It tracks the connection and processes incoming messages.
        
        Args:
            websocket: The WebSocket connection object (path available as websocket.path)
        """
        # Add connection to tracking set
        self.connections.add(websocket)
        remote_addr = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        path = getattr(websocket, 'path', '/')
        self.logger.info(f"New connection from {remote_addr} (path: {path})")
        
        try:
            # Process messages until connection closes
            async for message in websocket:
                # Update statistics
                if isinstance(message, bytes):
                    self.stats['bytes'] += len(message)
                elif isinstance(message, str):
                    self.stats['bytes'] += len(message.encode('utf-8'))
                else:
                    self.stats['bytes'] += len(str(message).encode('utf-8'))
                
                self.stats['messages'] += 1
                
                # Process the message (implemented by subclasses)
                await self._process_message(message)
        
        except websockets.exceptions.ConnectionClosed:
            self.logger.debug(f"Connection from {remote_addr} closed normally")
        except Exception as e:
            self.logger.error(f"Error handling connection from {remote_addr}: {e}")
        finally:
            # Remove connection from tracking set
            self.connections.discard(websocket)
            self.logger.info(f"Connection from {remote_addr} removed (total: {len(self.connections)})")
    
    @abstractmethod
    async def _process_message(self, message) -> None:
        """
        Process an incoming message from a WebSocket client.
        
        This method must be implemented by subclasses to handle
        sensor-specific message processing.
        
        Args:
            message: The message received from the client (bytes or str)
        """
        pass
    
    def get_stats(self) -> dict:
        """
        Get current statistics.
        
        Returns:
            Dictionary with 'messages' and 'bytes' counters
        """
        return {
            'messages': self.stats['messages'],
            'bytes': self.stats['bytes'],
            'connections': len(self.connections)
        }
    
    def reset_stats(self) -> None:
        """Reset statistics counters."""
        self.stats['messages'] = 0
        self.stats['bytes'] = 0
        self.logger.debug("Statistics reset")
