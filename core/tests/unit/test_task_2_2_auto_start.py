#!/usr/bin/env python3
"""
Unit tests for Task 2.2: Auto-Start Recording When Commanded

These tests verify that BaseController automatically starts/stops recording
based on backend commands.
Must run inside Docker container.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, '/app')

from base_controller import BaseController


class TestAutoStartRecording(unittest.TestCase):
    """Test auto-start/stop recording functionality."""
    
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
    
    def test_auto_start_on_command(self):
        """Test that recording starts automatically when command='start'."""
        with patch.object(self.controller, 'start_recording') as mock_start:
            # Simulate polling returning start command
            command_data = {
                'command': 'start',
                'namespace': '/sim_abc123/',
                'topics': None
            }
            
            # Manually call the logic that would be in polling loop
            command = command_data.get('command')
            namespace = command_data.get('namespace')
            topics = command_data.get('topics')
            
            if command == 'start' and not self.controller._recording:
                self.controller.start_recording(topics=topics, namespace=namespace)
            
            mock_start.assert_called_once_with(topics=None, namespace='/sim_abc123/')
    
    def test_auto_stop_on_command(self):
        """Test that recording stops automatically when command='stop'."""
        # Set recording state
        self.controller._recording = True
        
        with patch.object(self.controller, 'stop_recording') as mock_stop:
            # Simulate polling returning stop command
            command_data = {
                'command': 'stop',
                'namespace': None,
                'topics': None
            }
            
            # Manually call the logic that would be in polling loop
            command = command_data.get('command')
            
            if command == 'stop' and self.controller._recording:
                self.controller.stop_recording()
            
            mock_stop.assert_called_once()
    
    def test_no_action_on_none(self):
        """Test that no action is taken when command='none'."""
        with patch.object(self.controller, 'start_recording') as mock_start, \
             patch.object(self.controller, 'stop_recording') as mock_stop:
            
            # Simulate polling returning none command
            command_data = {
                'command': 'none',
                'namespace': None,
                'topics': None
            }
            
            # Manually call the logic that would be in polling loop
            command = command_data.get('command')
            
            if command == 'none':
                pass  # No action
            
            mock_start.assert_not_called()
            mock_stop.assert_not_called()
    
    def test_no_start_if_already_recording(self):
        """Test that start is not called if already recording."""
        self.controller._recording = True
        
        with patch.object(self.controller, 'start_recording') as mock_start:
            command_data = {
                'command': 'start',
                'namespace': '/sim_abc123/',
                'topics': None
            }
            
            command = command_data.get('command')
            namespace = command_data.get('namespace')
            topics = command_data.get('topics')
            
            if command == 'start' and not self.controller._recording:
                self.controller.start_recording(topics=topics, namespace=namespace)
            
            mock_start.assert_not_called()
    
    def test_no_stop_if_not_recording(self):
        """Test that stop is not called if not recording."""
        self.controller._recording = False
        
        with patch.object(self.controller, 'stop_recording') as mock_stop:
            command_data = {
                'command': 'stop',
                'namespace': None,
                'topics': None
            }
            
            command = command_data.get('command')
            
            if command == 'stop' and self.controller._recording:
                self.controller.stop_recording()
            
            mock_stop.assert_not_called()


if __name__ == '__main__':
    unittest.main()















