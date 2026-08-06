#!/usr/bin/env python3
"""
Unit tests for Task 2.1: Recording Command Polling

These tests verify that BaseController polls for recording commands correctly.
Must run inside Docker container.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, '/app')

# Import requests before base_controller to allow proper patching
import requests

from base_controller import BaseController


class TestRecordingPolling(unittest.TestCase):
    """Test recording command polling functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.controller = BaseController(
            controller_code='TEST123',
            backend_url='http://localhost:5000'
        )
        
        # Mock API client
        self.mock_api_client = Mock()
        self.mock_api_client.session_id = 'test-session-123'
        self.mock_api_client.api_token = 'test-token-456'
        self.controller._api_client = self.mock_api_client
    
    def test_poll_recording_command_start(self):
        """Test polling returns start command."""
        with patch('requests.get') as mock_get:
            mock_response = Mock()
            mock_response.json.return_value = {
                'command': 'start',
                'namespace': '/sim_abc123/',
                'topics': None
            }
            mock_response.raise_for_status = Mock()
            mock_get.return_value = mock_response
            
            result = self.controller.poll_recording_command()
            
            self.assertIsNotNone(result)
            self.assertEqual(result['command'], 'start')
            self.assertEqual(result['namespace'], '/sim_abc123/')
            self.assertIsNone(result['topics'])
            
            # Verify request was made correctly
            mock_get.assert_called_once()
            call_args = mock_get.call_args
            self.assertIn('session_id', call_args[1]['params'])
            self.assertIn('api_token', call_args[1]['params'])
    
    def test_poll_recording_command_none(self):
        """Test polling returns none command."""
        with patch('requests.get') as mock_get:
            mock_response = Mock()
            mock_response.json.return_value = {
                'command': 'none',
                'namespace': None,
                'topics': None
            }
            mock_response.raise_for_status = Mock()
            mock_get.return_value = mock_response
            
            result = self.controller.poll_recording_command()
            
            self.assertIsNotNone(result)
            self.assertEqual(result['command'], 'none')
    
    def test_poll_recording_command_error(self):
        """Test polling handles errors gracefully."""
        with patch('requests.get') as mock_get:
            mock_get.side_effect = Exception("Network error")
            
            result = self.controller.poll_recording_command()
            
            self.assertIsNone(result)
    
    def test_polling_loop_runs(self):
        """Test that polling loop starts and runs."""
        self.controller._polling_active = True
        
        with patch.object(self.controller, 'poll_recording_command') as mock_poll:
            mock_poll.return_value = {'command': 'none'}
            
            # Start polling thread
            import threading
            import time
            
            thread = threading.Thread(
                target=self.controller._poll_recording_loop,
                daemon=True
            )
            thread.start()
            
            # Wait a bit for polling to happen
            time.sleep(0.5)
            
            # Stop polling
            self.controller._polling_active = False
            thread.join(timeout=2.0)
            
            # Verify polling was called
            self.assertGreater(mock_poll.call_count, 0)
    
    def test_polling_starts_on_connect(self):
        """Test that polling starts automatically on connect."""
        with patch('base_controller.MavsimAPIClient.from_handshake') as mock_handshake, \
             patch('base_controller.MavsimController') as mock_controller_class:
            
            # Mock handshake response
            mock_api_client = Mock()
            mock_api_client.session_id = 'test-session'
            mock_api_client.api_token = 'test-token'
            
            mock_handshake_data = {
                'sessionId': 'test-session',
                'apiToken': 'test-token',
                'rosbridgeUrl': 'ws://localhost:9090',
                'namespace': '/sim_test/',
                'vesselName': 'vessel_01'
            }
            mock_handshake.return_value = (mock_api_client, mock_handshake_data)
            
            # Mock controller
            mock_controller = Mock()
            mock_controller.connect.return_value = True
            mock_controller_class.return_value = mock_controller
            
            # Mock polling
            with patch.object(self.controller, '_start_polling') as mock_start_polling:
                result = self.controller.connect()
                
                self.assertTrue(result)
                mock_start_polling.assert_called_once()


if __name__ == '__main__':
    unittest.main()

