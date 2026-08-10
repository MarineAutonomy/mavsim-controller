"""
Unit tests for Task 1.7: Command-Line Interface

Tests verify that the CLI:
- Parses arguments correctly (defaults and custom values)
- Handles all command-line options
- Validates argument types
"""

import pytest
import sys
from mavsim_sensor_bridge.cli import parse_args


def test_cli_default_args():
    """Test that CLI parses default arguments correctly."""
    args = parse_args([])
    
    assert args.camera_port == 8765
    assert args.verbose is False
    assert args.stats_interval == 0


def test_cli_custom_args():
    """Test that CLI parses custom arguments correctly."""
    args = parse_args(['--camera-port', '9000', '--verbose'])
    
    assert args.camera_port == 9000
    assert args.verbose is True
    assert args.stats_interval == 0


def test_cli_camera_port():
    """Test --camera-port option."""
    args = parse_args(['--camera-port', '8080'])
    assert args.camera_port == 8080
    
    args = parse_args(['--camera-port', '12345'])
    assert args.camera_port == 12345


def test_cli_verbose_flag():
    """Test --verbose flag."""
    args = parse_args(['--verbose'])
    assert args.verbose is True
    
    args = parse_args([])
    assert args.verbose is False


def test_cli_stats_interval():
    """Test --stats-interval option."""
    args = parse_args(['--stats-interval', '5'])
    assert args.stats_interval == 5.0
    
    args = parse_args(['--stats-interval', '10.5'])
    assert args.stats_interval == 10.5
    
    args = parse_args(['--stats-interval', '0'])
    assert args.stats_interval == 0.0


def test_cli_all_options():
    """Test all options together."""
    args = parse_args([
        '--camera-port', '9000',
        '--verbose',
        '--stats-interval', '5'
    ])
    
    assert args.camera_port == 9000
    assert args.verbose is True
    assert args.stats_interval == 5.0


def test_cli_camera_port_type():
    """Test that --camera-port requires an integer."""
    with pytest.raises(SystemExit):
        parse_args(['--camera-port', 'not-a-number'])


def test_cli_stats_interval_type():
    """Test that --stats-interval accepts float values."""
    args = parse_args(['--stats-interval', '5.5'])
    assert args.stats_interval == 5.5
    
    args = parse_args(['--stats-interval', '10'])
    assert args.stats_interval == 10.0


def test_cli_help():
    """Test that --help option works."""
    with pytest.raises(SystemExit) as exc_info:
        parse_args(['--help'])
    
    # argparse exits with code 0 when --help is used
    assert exc_info.value.code == 0
