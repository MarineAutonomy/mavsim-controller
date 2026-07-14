#!/usr/bin/env python3
"""
Entry Point for mavsim Controller

This script auto-discovers and runs client controller code.
It looks for:
1. my_controller.py (mounted by client)
2. controller.py (fallback)
3. Any file matching *_controller.py (fallback)

The client code should define a class that inherits from BaseController
and implements the control_loop method.

Author: mavsim Team
License: MIT
"""

import importlib.util
import logging
import os
import sys
from pathlib import Path

from base_controller import BaseController

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def find_controller_file() -> Path:
    """
    Find client controller file.
    
    Looks for files in order:
    1. my_controller.py
    2. controller.py
    3. Any *_controller.py file
    
    Returns:
        Path to controller file
        
    Raises:
        FileNotFoundError: If no controller file found
    """
    # Search user_code first (mounted controller), then /app
    search_dirs = [Path('/app/user_code'), Path('/app')]
    
    for app_dir in search_dirs:
        # Priority 1: my_controller.py
        candidate = app_dir / 'my_controller.py'
        if candidate.exists():
            logger.info(f"Found controller file: {candidate}")
            return candidate
        
        # Priority 2: controller.py
        candidate = app_dir / 'controller.py'
        if candidate.exists():
            logger.info(f"Found controller file: {candidate}")
            return candidate
        
        # Priority 3: Any *_controller.py file (exclude entry point and base)
        for file in app_dir.glob('*_controller.py'):
            if file.name not in ('base_controller.py', 'python_controller.py', 'run_controller.py'):
                logger.info(f"Found controller file: {file}")
                return file
    
    raise FileNotFoundError(
        "No controller file found. Please create one of:\n"
        "  - my_controller.py (preferred)\n"
        "  - controller.py\n"
        "  - or any *_controller.py file\n"
        "\n"
        "Place the file in the current directory and run:\n"
        "  ./start.sh <controller-code>"
    )


def load_controller_class(controller_file: Path) -> type:
    """
    Load controller class from file.
    
    Args:
        controller_file: Path to controller file
        
    Returns:
        Controller class (subclass of BaseController)
        
    Raises:
        ImportError: If controller class not found
    """
    module_name = controller_file.stem

    # Ensure the controller's directory is on sys.path so sibling imports work
    controller_dir = str(controller_file.parent)
    if controller_dir not in sys.path:
        sys.path.insert(0, controller_dir)

    spec = importlib.util.spec_from_file_location(module_name, controller_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"Failed to load module from {controller_file}")
    
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    
    # Find controller class (subclass of BaseController)
    controller_class = None
    for name in dir(module):
        obj = getattr(module, name)
        if (isinstance(obj, type) and 
            issubclass(obj, BaseController) and 
            obj != BaseController):
            controller_class = obj
            logger.info(f"Found controller class: {name}")
            break
    
    if controller_class is None:
        raise ImportError(
            f"No controller class found in {controller_file}. "
            f"Please define a class that inherits from BaseController."
        )
    
    return controller_class


def _load_token(path: str) -> dict:
    """Load and validate a controller token JSON file."""
    import json
    with open(path, 'r') as f:
        token = json.load(f)
    required = ('session_id', 'api_token', 'controller_code', 'backend_url', 'vessels')
    missing = [k for k in required if k not in token]
    if missing:
        raise ValueError(f"Token file missing required fields: {', '.join(missing)}")
    if not isinstance(token['vessels'], list) or len(token['vessels']) == 0:
        raise ValueError("Token 'vessels' must be a non-empty list")
    return token


def main():
    """Main entry point."""
    try:
        # Find controller file
        controller_file = find_controller_file()
        
        # Load controller class
        controller_class = load_controller_class(controller_file)
        
        import argparse
        parser = argparse.ArgumentParser(description='mavsim Controller')
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument('--code', help='Controller code from simulation UI')
        group.add_argument('--token', help='Path to controller token JSON file')
        parser.add_argument('--frontend-url', default='http://localhost:5173',
                           help='Where the MAVSim frontend is hosted (default: http://localhost:5173) - '
                                'override with the real address whenever the bridge isn\'t on the same '
                                'machine as the frontend. Used to launch the headless sensor observer, '
                                'see plans/plan_headless_observer.md')
        parser.add_argument('--backend-url', default='http://localhost:5000',
                           help='Backend URL (default: http://localhost:5000)')
        parser.add_argument('--vessel-name', help='Vessel name (auto-assigned if not specified)')
        parser.add_argument('--camera-port', type=int, default=None,
                           help='Camera sensor port override (default: auto per vessel)')
        parser.add_argument('--lidar-port', type=int, default=None,
                           help='Lidar sensor port override (default: auto per vessel)')
        parser.add_argument('--sensor-base-port', type=int, default=7000,
                           help='Base port for auto sensor bridge ports (default: 7000)')
        parser.add_argument('--rate', type=float, default=10.0,
                           help='Control loop rate in Hz (default: 10.0)')
        parser.add_argument('--rosbridge-port', type=int, default=9090,
                           help='Local rosbridge websocket port for the ROS2 topic visualizer '
                                '(default: 9090)')
        parser.add_argument('--visualizer-port', type=int, default=8899,
                           help='Local ROS2 topic visualizer port (default: 8899)')

        args = parser.parse_args()

        if args.token:
            token = _load_token(args.token)
            logger.info(
                f"Loaded token: session={token['session_id'][:8]}, "
                f"vessels={token['vessels']}"
            )
            controller = controller_class(
                controller_code=token['controller_code'],
                frontend_url=token.get('frontend_url', args.frontend_url),
                backend_url=token.get('backend_url', args.backend_url),
                vessel_name=None,  # multi-vessel from token
                camera_port=args.camera_port,
                lidar_port=args.lidar_port,
                sensor_base_port=args.sensor_base_port,
                token=token,
                rosbridge_port=args.rosbridge_port,
                visualizer_port=args.visualizer_port,
            )
        else:
            controller = controller_class(
                controller_code=args.code,
                frontend_url=args.frontend_url,
                backend_url=args.backend_url,
                vessel_name=args.vessel_name,
                camera_port=args.camera_port,
                lidar_port=args.lidar_port,
                sensor_base_port=args.sensor_base_port,
                rosbridge_port=args.rosbridge_port,
                visualizer_port=args.visualizer_port,
            )
        
        logger.info("Starting controller...")
        controller.run(rate_hz=args.rate)
        
    except FileNotFoundError as e:
        logger.error(str(e))
        sys.exit(1)
    except ImportError as e:
        logger.error(str(e))
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
