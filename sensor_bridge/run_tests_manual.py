#!/usr/bin/env python3
"""
Manual test runner for Task 1.1
Simulates pytest behavior without requiring pytest to be installed
"""

import sys
import importlib.util

def run_test(test_name, test_func):
    """Run a single test and report results"""
    try:
        test_func()
        print(f"✓ {test_name}")
        return True
    except AssertionError as e:
        print(f"✗ {test_name}: {e}")
        return False
    except Exception as e:
        print(f"✗ {test_name}: Unexpected error - {e}")
        return False

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
    """Test that the package is installable."""
    import importlib
    
    # Try to import the package
    try:
        import mavsim_sensor_bridge
        assert mavsim_sensor_bridge is not None
    except ImportError as e:
        raise AssertionError(f"Package could not be imported: {e}")
    
    # Verify package metadata
    assert hasattr(mavsim_sensor_bridge, '__version__')
    assert hasattr(mavsim_sensor_bridge, 'SensorBridge')

def test_package_all_exports():
    """Test that __all__ is defined and contains expected exports."""
    from mavsim_sensor_bridge import __all__
    
    assert '__all__' in dir(__import__('mavsim_sensor_bridge'))
    assert 'SensorBridge' in __all__
    assert '__version__' in __all__

def main():
    """Run all tests"""
    print("Running Task 1.1 Manual Tests")
    print("=" * 50)
    
    # Add current directory to path
    sys.path.insert(0, '.')
    
    tests = [
        ("test_package_imports", test_package_imports),
        ("test_package_version_format", test_package_version_format),
        ("test_sensor_bridge_class_exists", test_sensor_bridge_class_exists),
        ("test_package_installable", test_package_installable),
        ("test_package_all_exports", test_package_all_exports),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        if run_test(test_name, test_func):
            passed += 1
        else:
            failed += 1
    
    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    
    if failed == 0:
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed")
        return 1

if __name__ == "__main__":
    sys.exit(main())
