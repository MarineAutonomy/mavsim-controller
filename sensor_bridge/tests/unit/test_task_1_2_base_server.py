"""
Unit tests for Task 1.2: Base Sensor Server Class

Tests verify that the BaseSensorServer abstract class works correctly:
- Server starts and stops cleanly
- Connection tracking works
- Statistics are tracked correctly
- Logging is configurable
"""

import asyncio
import logging
import pytest
import websockets
from mavsim_sensor_bridge.servers.base import BaseSensorServer


class ConcreteTestServer(BaseSensorServer):
    """
    Concrete implementation of BaseSensorServer for testing purposes.
    
    This class implements the abstract _process_message method to allow
    testing of the base class functionality.
    """
    
    def __init__(self, port: int, name: str = "test", log_level: int = logging.WARNING):
        super().__init__(port, name, log_level)
        self.received_messages = []
    
    async def _process_message(self, message) -> None:
        """Store received messages for testing."""
        self.received_messages.append(message)


@pytest.mark.asyncio
async def test_server_starts_and_stops():
    """Test that server can start and stop cleanly."""
    server = ConcreteTestServer(port=18765, name="test")
    
    # Server should not be running initially
    assert not server.is_running
    
    # Start server in background task
    server_task = asyncio.create_task(server.start())
    
    # Wait a bit for server to start
    await asyncio.sleep(0.1)
    
    # Server should be running
    assert server.is_running
    
    # Stop the server
    await server.stop()
    
    # Wait for server task to complete
    await asyncio.sleep(0.1)
    
    # Server should not be running anymore
    assert not server.is_running


@pytest.mark.asyncio
async def test_server_tracks_connections():
    """Test that server correctly tracks WebSocket connections."""
    server = ConcreteTestServer(port=18766, name="test")
    
    # Start server
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)
    
    try:
        # Connect a client
        async with websockets.connect("ws://localhost:18766") as ws:
            # Wait a bit for connection to be registered
            await asyncio.sleep(0.1)
            
            # Should have one connection
            assert len(server.connections) == 1
            
            # Send a test message
            await ws.send(b"test message")
            await asyncio.sleep(0.1)
            
            # Message should be received
            assert len(server.received_messages) == 1
        
        # Wait a bit for connection to be removed
        await asyncio.sleep(0.1)
        
        # Connection should be removed after disconnect
        assert len(server.connections) == 0
    
    finally:
        await server.stop()
        await asyncio.sleep(0.1)


@pytest.mark.asyncio
async def test_server_tracks_statistics():
    """Test that server correctly tracks message and byte statistics."""
    server = ConcreteTestServer(port=18767, name="test")
    
    # Start server
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)
    
    try:
        async with websockets.connect("ws://localhost:18767") as ws:
            # Send multiple messages
            test_messages = [
                b"message1",
                b"message2",
                b"message3"
            ]
            
            for msg in test_messages:
                await ws.send(msg)
                await asyncio.sleep(0.05)
            
            # Wait for all messages to be processed
            await asyncio.sleep(0.1)
            
            # Check statistics
            stats = server.get_stats()
            assert stats['messages'] == 3
            assert stats['bytes'] == len(b"message1") + len(b"message2") + len(b"message3")
            assert stats['connections'] == 1
    
    finally:
        await server.stop()
        await asyncio.sleep(0.1)


@pytest.mark.asyncio
async def test_server_reset_stats():
    """Test that statistics can be reset."""
    server = ConcreteTestServer(port=18768, name="test")
    
    # Start server
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)
    
    try:
        async with websockets.connect("ws://localhost:18768") as ws:
            # Send a message
            await ws.send(b"test")
            await asyncio.sleep(0.1)
            
            # Verify stats exist
            stats = server.get_stats()
            assert stats['messages'] > 0
            
            # Reset stats
            server.reset_stats()
            
            # Verify stats are reset
            stats = server.get_stats()
            assert stats['messages'] == 0
            assert stats['bytes'] == 0
    
    finally:
        await server.stop()
        await asyncio.sleep(0.1)


@pytest.mark.asyncio
async def test_server_multiple_connections():
    """Test that server can handle multiple concurrent connections."""
    server = ConcreteTestServer(port=18769, name="test")
    
    # Start server
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)
    
    try:
        # Connect multiple clients
        async with websockets.connect("ws://localhost:18769") as ws1:
            await asyncio.sleep(0.05)
            
            async with websockets.connect("ws://localhost:18769") as ws2:
                await asyncio.sleep(0.05)
                
                async with websockets.connect("ws://localhost:18769") as ws3:
                    await asyncio.sleep(0.1)
                    
                    # Should have 3 connections
                    assert len(server.connections) == 3
                    
                    # Send messages from all connections
                    await ws1.send(b"msg1")
                    await ws2.send(b"msg2")
                    await ws3.send(b"msg3")
                    await asyncio.sleep(0.1)
                    
                    # Should have received 3 messages
                    assert len(server.received_messages) == 3
        
        # Wait for connections to close
        await asyncio.sleep(0.1)
        
        # All connections should be removed
        assert len(server.connections) == 0
    
    finally:
        await server.stop()
        await asyncio.sleep(0.1)


@pytest.mark.asyncio
async def test_server_string_messages():
    """Test that server handles string messages correctly."""
    server = ConcreteTestServer(port=18770, name="test")
    
    # Start server
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)
    
    try:
        async with websockets.connect("ws://localhost:18770") as ws:
            # Send string message
            await ws.send("test string message")
            await asyncio.sleep(0.1)
            
            # Message should be received
            assert len(server.received_messages) == 1
            assert server.received_messages[0] == "test string message"
            
            # Bytes should be counted correctly (UTF-8 encoding)
            stats = server.get_stats()
            expected_bytes = len("test string message".encode('utf-8'))
            assert stats['bytes'] == expected_bytes
    
    finally:
        await server.stop()
        await asyncio.sleep(0.1)


def test_server_logging_configuration():
    """Test that logging level can be configured."""
    # Create server with DEBUG logging
    server_debug = ConcreteTestServer(port=18771, name="test_debug", log_level=logging.DEBUG)
    assert server_debug.logger.level == logging.DEBUG
    
    # Create server with WARNING logging
    server_warning = ConcreteTestServer(port=18772, name="test_warning", log_level=logging.WARNING)
    assert server_warning.logger.level == logging.WARNING


def test_server_initialization():
    """Test that server initializes with correct attributes."""
    server = ConcreteTestServer(port=9999, name="test_server")
    
    assert server.port == 9999
    assert server.name == "test_server"
    assert len(server.connections) == 0
    assert server.stats['messages'] == 0
    assert server.stats['bytes'] == 0
    assert not server.is_running
    assert server.logger is not None


@pytest.mark.asyncio
async def test_server_stop_when_not_running():
    """Test that stopping a non-running server doesn't raise errors."""
    server = ConcreteTestServer(port=18773, name="test")
    
    # Should not raise an error
    await server.stop()
    
    # Should still not be running
    assert not server.is_running


@pytest.mark.asyncio
async def test_server_start_twice():
    """Test that starting an already-running server doesn't cause issues."""
    server = ConcreteTestServer(port=18774, name="test")
    
    # Start server
    server_task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)
    
    try:
        # Try to start again (should just log warning and return)
        await server.start()
        
        # Server should still be running
        assert server.is_running
    
    finally:
        await server.stop()
        await asyncio.sleep(0.1)
