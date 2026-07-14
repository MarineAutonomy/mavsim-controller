#!/usr/bin/env python3
"""
Unit tests for Task 2.4: Recording Status Reporting

These tests verify that recording status is reported to backend correctly.
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
from python_controller import MavsimAPIClient


class TestStatusReporting(unittest.TestCase):
    """Test recording status reporting functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.controller = BaseController(
            controller_code='TEST123',
            backend_url='http://localhost:5000'
        )
        
        # Mock API client
        self.mock_api_client = Mock(spec=MavsimAPIClient)
        self.mock_api_client.session_id = 'test-session-123'
        self.mock_api_client.api_token = 'test-token-456'
        self.controller._api_client = self.mock_api_client
    
    def test_report_recording_started(self):
        """Test that recording started status is reported."""
        with patch.object(self.mock_api_client, 'report_recording_status') as mock_report:
            self.controller._report_recording_status('recording', ['/topic1', '/topic2'])
            
            mock_report.assert_called_once_with(
                status='recording',
                topics=['/topic1', '/topic2'],
                error=None
            )
    
    def test_report_recording_stopped(self):
        """Test that recording stopped status is reported."""
        with patch.object(self.mock_api_client, 'report_recording_status') as mock_report:
            self.controller._report_recording_status('stopped', ['/topic1'])
            
            mock_report.assert_called_once_with(
                status='stopped',
                topics=['/topic1'],
                error=None
            )
    
    def test_report_recording_error(self):
        """Test that recording error status is reported."""
        with patch.object(self.mock_api_client, 'report_recording_status') as mock_report:
            error_msg = "Failed to start recording"
            self.controller._report_recording_status('error', [], error=error_msg)
            
            mock_report.assert_called_once_with(
                status='error',
                topics=None,  # Empty list is converted to None in report_recording_status
                error=error_msg
            )
    
    def test_report_no_api_client(self):
        """Test that reporting is skipped if no API client."""
        self.controller._api_client = None
        
        # Should not raise exception
        self.controller._report_recording_status('recording', [])
    
    def test_report_handles_exception(self):
        """Test that reporting handles exceptions gracefully."""
        self.mock_api_client.report_recording_status.side_effect = Exception("Network error")
        
        # Should not raise exception
        with patch('base_controller.logger') as mock_logger:
            self.controller._report_recording_status('recording', [])
            mock_logger.warning.assert_called_once()


class TestMavsimAPIClientStatusReporting(unittest.TestCase):
    """Test MavsimAPIClient.report_recording_status method."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.client = MavsimAPIClient(
            backend_url='http://localhost:5000',
            session_id='test-session-123',
            api_token='test-token-456'
        )
    
    def test_report_recording_status_success(self):
        """Test successful status reporting."""
        with patch('python_controller.requests.post') as mock_post:
            mock_response = Mock()
            mock_response.status_code = 200
            mock_response.json.return_value = {'success': True}
            mock_post.return_value = mock_response
            
            result = self.client.report_recording_status('recording', ['/topic1'])
            
            self.assertTrue(result)
            mock_post.assert_called_once()
            call_args = mock_post.call_args
            self.assertEqual(call_args[0][0], 'http://localhost:5000/api/simulation/recording/status')
            self.assertIn('sessionId', call_args[1]['json'])
            self.assertIn('apiToken', call_args[1]['json'])
            self.assertEqual(call_args[1]['json']['status'], 'recording')
    
    def test_report_recording_status_with_error(self):
        """Test status reporting with error message."""
        with patch('python_controller.requests.post') as mock_post:
            mock_response = Mock()
            mock_response.status_code = 200
            mock_response.json.return_value = {'success': True}
            mock_post.return_value = mock_response
            
            result = self.client.report_recording_status(
                'error',
                topics=['/topic1'],
                error='Test error message'
            )
            
            self.assertTrue(result)
            call_args = mock_post.call_args
            self.assertEqual(call_args[1]['json']['status'], 'error')
            self.assertEqual(call_args[1]['json']['error'], 'Test error message')
    
    def test_report_recording_status_failure(self):
        """Test status reporting failure handling."""
        with patch('python_controller.requests.post') as mock_post:
            mock_response = Mock()
            mock_response.status_code = 400
            mock_response.json.return_value = {'error': 'Invalid request'}
            mock_post.return_value = mock_response
            
            result = self.client.report_recording_status('recording', [])
            
            self.assertFalse(result)
    
    def test_report_recording_status_no_auth(self):
        """Test that reporting fails without authentication."""
        self.client.session_id = None
        self.client.api_token = None
        
        result = self.client.report_recording_status('recording', [])
        
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()

