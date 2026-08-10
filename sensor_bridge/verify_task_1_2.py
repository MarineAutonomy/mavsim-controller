#!/usr/bin/env python3
"""
Verification script for Task 1.2: Base Sensor Server Class

This script verifies that the implementation matches all requirements
from the plan verification checklist.
"""

import sys
import inspect
from abc import ABC

# Import the BaseSensorServer class
from mavsim_sensor_bridge.servers.base import BaseSensorServer


def verify_deliverables():
    """Verify all deliverables from the plan."""
    print("=" * 60)
    print("TASK 1.2 VERIFICATION CHECKLIST")
    print("=" * 60)
    
    checks_passed = 0
    checks_total = 0
    
    # 1. Create BaseSensorServer abstract class
    print("\n1. BaseSensorServer Abstract Class")
    print("-" * 60)
    checks_total += 1
    if BaseSensorServer:
        print("   ✓ BaseSensorServer class exists")
        checks_passed += 1
    else:
        print("   ✗ BaseSensorServer class missing")
    
    checks_total += 1
    if issubclass(BaseSensorServer, ABC):
        print("   ✓ Inherits from ABC")
        checks_passed += 1
    else:
        print("   ✗ Does not inherit from ABC")
    
    checks_total += 1
    if "_process_message" in BaseSensorServer.__abstractmethods__:
        print("   ✓ Has abstract _process_message method")
        checks_passed += 1
    else:
        print("   ✗ Missing abstract _process_message method")
    
    # 2. Check __init__ signature
    print("\n2. __init__ Method Signature")
    print("-" * 60)
    init_sig = inspect.signature(BaseSensorServer.__init__)
    params = list(init_sig.parameters.keys())[1:]  # Skip 'self'
    
    checks_total += 1
    if "port" in params:
        print(f"   ✓ Has 'port' parameter (params: {params})")
        checks_passed += 1
    else:
        print(f"   ✗ Missing 'port' parameter (params: {params})")
    
    checks_total += 1
    if "name" in params:
        print(f"   ✓ Has 'name' parameter")
        checks_passed += 1
    else:
        print(f"   ✗ Missing 'name' parameter")
    
    # 3. Check required attributes
    print("\n3. Required Attributes")
    print("-" * 60)
    
    # Create a test instance to check attributes
    class TestServer(BaseSensorServer):
        async def _process_message(self, message):
            pass
    
    test = TestServer(port=9999, name='test')
    
    checks_total += 1
    if hasattr(test, 'port') and test.port == 9999:
        print("   ✓ Has 'port' attribute")
        checks_passed += 1
    else:
        print("   ✗ Missing or incorrect 'port' attribute")
    
    checks_total += 1
    if hasattr(test, 'name') and test.name == 'test':
        print("   ✓ Has 'name' attribute")
        checks_passed += 1
    else:
        print("   ✗ Missing or incorrect 'name' attribute")
    
    checks_total += 1
    if hasattr(test, 'connections') and isinstance(test.connections, set):
        print("   ✓ Has 'connections' attribute (set)")
        checks_passed += 1
    else:
        print("   ✗ Missing or incorrect 'connections' attribute")
    
    checks_total += 1
    if hasattr(test, 'stats') and isinstance(test.stats, dict):
        print("   ✓ Has 'stats' attribute (dict)")
        checks_passed += 1
    else:
        print("   ✗ Missing or incorrect 'stats' attribute")
    
    checks_total += 1
    if 'messages' in test.stats and 'bytes' in test.stats:
        print("   ✓ stats dict has 'messages' and 'bytes' keys")
        checks_passed += 1
    else:
        print("   ✗ stats dict missing required keys")
    
    # 4. Check async WebSocket server lifecycle
    print("\n4. Async WebSocket Server Lifecycle")
    print("-" * 60)
    
    checks_total += 1
    if hasattr(BaseSensorServer, 'start'):
        print("   ✓ Has 'start()' method")
        checks_passed += 1
    else:
        print("   ✗ Missing 'start()' method")
    
    checks_total += 1
    if inspect.iscoroutinefunction(BaseSensorServer.start):
        print("   ✓ start() is async")
        checks_passed += 1
    else:
        print("   ✗ start() is not async")
    
    checks_total += 1
    if hasattr(BaseSensorServer, 'stop'):
        print("   ✓ Has 'stop()' method")
        checks_passed += 1
    else:
        print("   ✗ Missing 'stop()' method")
    
    checks_total += 1
    if inspect.iscoroutinefunction(BaseSensorServer.stop):
        print("   ✓ stop() is async")
        checks_passed += 1
    else:
        print("   ✗ stop() is not async")
    
    checks_total += 1
    if hasattr(BaseSensorServer, 'is_running'):
        print("   ✓ Has 'is_running' property")
        checks_passed += 1
    else:
        print("   ✗ Missing 'is_running' property")
    
    # 5. Check connection tracking
    print("\n5. Connection Tracking")
    print("-" * 60)
    
    checks_total += 1
    if hasattr(BaseSensorServer, '_handle_connection'):
        print("   ✓ Has '_handle_connection()' method")
        checks_passed += 1
    else:
        print("   ✗ Missing '_handle_connection()' method")
    
    checks_total += 1
    if inspect.iscoroutinefunction(BaseSensorServer._handle_connection):
        print("   ✓ _handle_connection() is async")
        checks_passed += 1
    else:
        print("   ✗ _handle_connection() is not async")
    
    # 6. Check statistics
    print("\n6. Basic Statistics")
    print("-" * 60)
    
    checks_total += 1
    if hasattr(test, 'get_stats'):
        print("   ✓ Has 'get_stats()' method")
        checks_passed += 1
    else:
        print("   ✗ Missing 'get_stats()' method")
    
    checks_total += 1
    if hasattr(test, 'reset_stats'):
        print("   ✓ Has 'reset_stats()' method")
        checks_passed += 1
    else:
        print("   ✗ Missing 'reset_stats()' method")
    
    # 7. Check logging
    print("\n7. Logging with Configurable Verbosity")
    print("-" * 60)
    
    checks_total += 1
    if hasattr(test, 'logger'):
        print("   ✓ Has 'logger' attribute")
        checks_passed += 1
    else:
        print("   ✗ Missing 'logger' attribute")
    
    checks_total += 1
    # Check that log_level parameter exists in __init__
    init_sig = inspect.signature(BaseSensorServer.__init__)
    if 'log_level' in init_sig.parameters:
        print("   ✓ __init__ accepts 'log_level' parameter")
        checks_passed += 1
    else:
        print("   ✗ __init__ does not accept 'log_level' parameter")
    
    # 8. Check code structure matches plan
    print("\n8. Code Structure Verification")
    print("-" * 60)
    
    checks_total += 1
    if hasattr(BaseSensorServer, '_process_message'):
        print("   ✓ Has '_process_message()' method")
        checks_passed += 1
    else:
        print("   ✗ Missing '_process_message()' method")
    
    checks_total += 1
    # inspect.isabstract() doesn't work correctly for async methods
    # Check if it's in __abstractmethods__ instead
    if "_process_message" in BaseSensorServer.__abstractmethods__:
        print("   ✓ _process_message() is abstract")
        checks_passed += 1
    else:
        print("   ✗ _process_message() is not abstract")
    
    # Summary
    print("\n" + "=" * 60)
    print("VERIFICATION SUMMARY")
    print("=" * 60)
    print(f"Checks passed: {checks_passed}/{checks_total}")
    print(f"Success rate: {100 * checks_passed / checks_total:.1f}%")
    
    if checks_passed == checks_total:
        print("\n✓ ALL VERIFICATION CHECKS PASSED!")
        return 0
    else:
        print(f"\n✗ {checks_total - checks_passed} CHECK(S) FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(verify_deliverables())
