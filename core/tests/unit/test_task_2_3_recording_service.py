#!/usr/bin/env python3
"""
Unit tests for Task 2.3: ROS2 Bag Recording Service

These tests verify that RecordingService can discover topics and record bags.
Must run inside Docker container with ROS2 Humble.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import subprocess

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, '/app')

from recording_service import RecordingService


class TestRecordingService(unittest.TestCase):
    """Test ROS2 bag recording service functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.service = RecordingService(bag_dir='/tmp/test_bags')
    
    def test_discover_namespace_topics(self):
        """Test topic discovery for namespace."""
        with patch('recording_service.subprocess.run') as mock_run:
            # Mock ros2 topic list output
            mock_result = Mock()
            mock_result.returncode = 0
            mock_result.stdout = '/sim_abc123/odometry\n/sim_abc123/actuator\n/sim_abc123/camera/image_raw\n/other/topic\n'
            mock_run.return_value = mock_result
            
            topics = self.service.discover_namespace_topics('/sim_abc123/')
            
            self.assertEqual(len(topics), 3)
            self.assertIn('/sim_abc123/odometry', topics)
            self.assertIn('/sim_abc123/actuator', topics)
            self.assertIn('/sim_abc123/camera/image_raw', topics)
            self.assertNotIn('/other/topic', topics)
    
    def test_discover_namespace_topics_empty(self):
        """Test topic discovery when no topics found."""
        with patch('recording_service.subprocess.run') as mock_run:
            mock_result = Mock()
            mock_result.returncode = 0
            mock_result.stdout = '/other/topic\n'
            mock_run.return_value = mock_result
            
            topics = self.service.discover_namespace_topics('/sim_abc123/')
            
            self.assertEqual(len(topics), 0)
    
    def test_discover_namespace_topics_error(self):
        """Test topic discovery handles errors."""
        with patch('recording_service.subprocess.run') as mock_run:
            mock_result = Mock()
            mock_result.returncode = 1
            mock_result.stderr = 'Error message'
            mock_run.return_value = mock_result
            
            topics = self.service.discover_namespace_topics('/sim_abc123/')
            
            self.assertEqual(len(topics), 0)
    
    def test_start_recording_with_topics(self):
        """Test starting recording with specific topics."""
        with patch('recording_service.subprocess.Popen') as mock_popen:
            mock_process = Mock()
            mock_process.poll.return_value = None  # Process running
            mock_popen.return_value = mock_process
            
            success = self.service.start_recording(
                namespace='/sim_abc123/',
                topics=['/sim_abc123/odometry', '/sim_abc123/actuator']
            )
            
            self.assertTrue(success)
            self.assertTrue(self.service.is_recording())
            self.assertEqual(len(self.service.get_topics()), 2)
            mock_popen.assert_called_once()
    
    def test_start_recording_auto_discover(self):
        """Test starting recording with auto-discovery."""
        with patch.object(self.service, 'discover_namespace_topics') as mock_discover, \
             patch('recording_service.subprocess.Popen') as mock_popen:
            
            mock_discover.return_value = ['/sim_abc123/odometry', '/sim_abc123/actuator']
            mock_process = Mock()
            mock_process.poll.return_value = None
            mock_popen.return_value = mock_process
            
            success = self.service.start_recording(
                namespace='/sim_abc123/',
                topics=None
            )
            
            self.assertTrue(success)
            mock_discover.assert_called_once_with('/sim_abc123/')
    
    def test_start_recording_already_recording(self):
        """Test that starting recording fails if already recording."""
        self.service._recording = True
        
        success = self.service.start_recording(
            namespace='/sim_abc123/',
            topics=['/topic1']
        )
        
        self.assertFalse(success)
    
    def test_stop_recording(self):
        """Test stopping recording."""
        self.service._recording = True
        self.service._bag_path = '/tmp/test_bags/test_bag'
        mock_process = Mock()
        self.service._bag_process = mock_process
        
        with patch('recording_service.Path.exists') as mock_exists:
            mock_exists.return_value = True
            
            bag_path = self.service.stop_recording()
            
            self.assertFalse(self.service.is_recording())
            self.assertIsNotNone(bag_path)
            mock_process.terminate.assert_called_once()
    
    def test_stop_recording_not_recording(self):
        """Test that stopping recording fails if not recording."""
        bag_path = self.service.stop_recording()
        
        self.assertIsNone(bag_path)
    
    def test_is_recording(self):
        """Test is_recording status."""
        self.assertFalse(self.service.is_recording())
        
        self.service._recording = True
        self.assertTrue(self.service.is_recording())


if __name__ == '__main__':
    unittest.main()















