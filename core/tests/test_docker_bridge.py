#!/usr/bin/env python3
"""
Test Task 2.6: Sensor Bridge in Examples Docker Container

This test verifies that the sensor bridge package is correctly installed
and can be imported in the examples Docker container.

Usage:
    # Run inside the examples container:
    docker run --rm mavsim-controller python -m pytest /app/tests/test_docker_bridge.py -v
    
    # Or run from host (requires container to be built):
    docker run --rm mavsim-controller python -c "from mavsim_sensor_bridge import SensorBridge; print('OK')"
"""

import subprocess
import sys
import unittest


class TestBridgeAvailableInContainer(unittest.TestCase):
    """Test that sensor bridge can be imported in examples container."""
    
    def test_bridge_import(self):
        """Verify sensor bridge can be imported."""
        try:
            from mavsim_sensor_bridge import SensorBridge, BridgeConfig
            from mavsim_sensor_bridge import __version__
            
            # Verify we can instantiate the bridge
            config = BridgeConfig()
            bridge = SensorBridge(config=config)
            
            self.assertIsNotNone(bridge)
            self.assertIsNotNone(config)
            print(f"✓ SensorBridge imported successfully (version {__version__})")
            
        except ImportError as e:
            self.fail(f"Failed to import sensor bridge: {e}")
    
    def test_bridge_config(self):
        """Verify bridge configuration can be created."""
        try:
            from mavsim_sensor_bridge import BridgeConfig
            
            config = BridgeConfig()
            
            # Verify default ports
            self.assertEqual(config.camera_port, 8765)
            self.assertEqual(config.lidar_port, 8766)
            self.assertEqual(config.sonar_port, 8767)
            self.assertEqual(config.depth_camera_port, 8768)
            self.assertEqual(config.auxiliary_port, 8769)
            
            print("✓ BridgeConfig created with correct default ports")
            
        except ImportError as e:
            self.fail(f"Failed to import BridgeConfig: {e}")


def test_bridge_available_in_container():
    """
    Standalone test function for direct execution.
    Verifies sensor bridge can be imported in examples container.
    """
    # This is the test from the plan - can be run directly
    result = subprocess.run([
        'docker', 'run', '--rm', 'mavlab/mavsim-controller:latest',
        'python', '-c', 'from mavsim_sensor_bridge import SensorBridge; print("OK")'
    ], capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"Error: {result.stderr}")
        return False
    
    if 'OK' not in result.stdout:
        print(f"Unexpected output: {result.stdout}")
        return False
    
    print("✓ Bridge available in container test passed")
    return True


if __name__ == '__main__':
    # If running inside container, run unittest
    # If running from host, run the docker test
    if '--docker-test' in sys.argv:
        # Run the docker-based test
        success = test_bridge_available_in_container()
        sys.exit(0 if success else 1)
    else:
        # Run unittest tests (when inside container)
        unittest.main(verbosity=2)

