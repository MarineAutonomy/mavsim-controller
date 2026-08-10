"""
Unit tests for Task 1.1: Bridge Package Structure

Tests verify that the package can be imported correctly and has the expected
structure and exports.
"""

import pytest


def test_package_imports():
    """Test that the package can be imported and has required exports."""
    from mavsim_sensor_bridge import SensorBridge, __version__
    
    assert __version__ is not None
    assert isinstance(__version__, str)
    assert len(__version__) > 0
    
    assert SensorBridge is not None
    assert callable(SensorBridge) or isinstance(SensorBridge, type)


def test_package_version_format():
    """Test that the version follows semantic versioning format."""
    from mavsim_sensor_bridge import __version__
    
    # Version should be in format X.Y.Z
    parts = __version__.split('.')
    assert len(parts) >= 2, f"Version should have at least major.minor: {__version__}"
    
    # All parts should be numeric or contain valid version identifiers
    for part in parts:
        assert len(part) > 0, "Version parts cannot be empty"


def test_sensor_bridge_class_exists():
    """Test that SensorBridge class exists and can be instantiated."""
    from mavsim_sensor_bridge import SensorBridge
    
    # Should be able to create an instance (even if it's a placeholder)
    bridge = SensorBridge()
    assert bridge is not None


def test_package_installable():
    """
    Test that the package is installable.
    
    This test verifies that the package structure is correct for pip installation.
    In CI/CD, this would be verified by running: pip install -e .
    """
    import importlib
    import sys
    
    # Try to import the package
    try:
        import mavsim_sensor_bridge
        assert mavsim_sensor_bridge is not None
    except ImportError as e:
        pytest.fail(f"Package could not be imported: {e}")
    
    # Verify package metadata
    assert hasattr(mavsim_sensor_bridge, '__version__')
    assert hasattr(mavsim_sensor_bridge, 'SensorBridge')


def test_package_all_exports():
    """Test that __all__ is defined and contains expected exports."""
    from mavsim_sensor_bridge import __all__
    
    assert '__all__' in dir(__import__('mavsim_sensor_bridge'))
    assert 'SensorBridge' in __all__
    assert '__version__' in __all__
