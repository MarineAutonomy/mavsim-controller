"""
Unit tests for Task 1.8: Bridge Configuration

Tests verify that BridgeConfig:
- Provides correct default ports and options
- Loads from YAML file (from_yaml)
- Loads from environment variables (from_env)
- Handles missing/invalid values gracefully
"""

import logging
import os
import pytest
from pathlib import Path

from mavsim_sensor_bridge.config import (
    BridgeConfig,
    DEFAULT_CAMERA_PORT,
    DEFAULT_LIDAR_PORT,
    DEFAULT_SONAR_PORT,
    DEFAULT_DEPTH_PORT,
    DEFAULT_AUXILIARY_PORT,
)


def test_default_config():
    """Test that default config has correct default ports and options."""
    config = BridgeConfig()
    assert config.camera_port == 8765
    assert config.lidar_port == 8766
    assert config.sonar_port == 8767
    assert config.depth_port == 8768
    assert config.auxiliary_port == 8769
    assert config.camera_enabled is True
    assert config.log_level == logging.INFO


def test_default_config_uses_constants():
    """Test that default config matches module constants."""
    config = BridgeConfig()
    assert config.camera_port == DEFAULT_CAMERA_PORT
    assert config.lidar_port == DEFAULT_LIDAR_PORT
    assert config.sonar_port == DEFAULT_SONAR_PORT
    assert config.depth_port == DEFAULT_DEPTH_PORT
    assert config.auxiliary_port == DEFAULT_AUXILIARY_PORT


def test_config_from_yaml(tmp_path):
    """Test loading config from YAML file."""
    yaml_file = tmp_path / "config.yaml"
    yaml_file.write_text("camera_port: 9000\n")
    config = BridgeConfig.from_yaml(yaml_file)
    assert config.camera_port == 9000
    # Other values remain default
    assert config.lidar_port == DEFAULT_LIDAR_PORT


def test_config_from_yaml_multiple_keys(tmp_path):
    """Test loading multiple keys from YAML."""
    yaml_file = tmp_path / "config.yaml"
    yaml_file.write_text(
        "camera_port: 9000\n"
        "lidar_port: 9001\n"
        "camera_enabled: false\n"
    )
    config = BridgeConfig.from_yaml(yaml_file)
    assert config.camera_port == 9000
    assert config.lidar_port == 9001
    assert config.camera_enabled is False
    assert config.sonar_port == DEFAULT_SONAR_PORT


def test_config_from_yaml_log_level(tmp_path):
    """Test loading log_level from YAML (string)."""
    yaml_file = tmp_path / "config.yaml"
    yaml_file.write_text("log_level: DEBUG\n")
    config = BridgeConfig.from_yaml(yaml_file)
    assert config.log_level == logging.DEBUG


def test_config_from_yaml_missing_file(tmp_path):
    """Test that from_yaml raises FileNotFoundError for missing file."""
    missing = tmp_path / "nonexistent.yaml"
    with pytest.raises(FileNotFoundError) as exc_info:
        BridgeConfig.from_yaml(missing)
    assert "not found" in str(exc_info.value) or str(missing) in str(exc_info.value)


def test_config_from_yaml_empty_file(tmp_path):
    """Test loading from empty YAML file returns defaults."""
    yaml_file = tmp_path / "config.yaml"
    yaml_file.write_text("")
    config = BridgeConfig.from_yaml(yaml_file)
    assert config.camera_port == DEFAULT_CAMERA_PORT
    assert config.lidar_port == DEFAULT_LIDAR_PORT


def test_config_from_env(monkeypatch):
    """Test loading config from environment variables."""
    monkeypatch.setenv("SENSOR_BRIDGE_CAMERA_PORT", "9001")
    config = BridgeConfig.from_env()
    assert config.camera_port == 9001
    # Other values remain default
    assert config.lidar_port == DEFAULT_LIDAR_PORT


def test_config_from_env_multiple(monkeypatch):
    """Test loading multiple env vars."""
    monkeypatch.setenv("SENSOR_BRIDGE_CAMERA_PORT", "9000")
    monkeypatch.setenv("SENSOR_BRIDGE_LIDAR_PORT", "9002")
    monkeypatch.setenv("SENSOR_BRIDGE_CAMERA_ENABLED", "false")
    config = BridgeConfig.from_env()
    assert config.camera_port == 9000
    assert config.lidar_port == 9002
    assert config.camera_enabled is False


def test_config_from_env_log_level(monkeypatch):
    """Test SENSOR_BRIDGE_LOG_LEVEL env var."""
    monkeypatch.setenv("SENSOR_BRIDGE_LOG_LEVEL", "DEBUG")
    config = BridgeConfig.from_env()
    assert config.log_level == logging.DEBUG


def test_config_from_env_camera_enabled_true(monkeypatch):
    """Test SENSOR_BRIDGE_CAMERA_ENABLED=true."""
    monkeypatch.setenv("SENSOR_BRIDGE_CAMERA_ENABLED", "true")
    config = BridgeConfig.from_env()
    assert config.camera_enabled is True


def test_config_from_env_camera_enabled_false(monkeypatch):
    """Test SENSOR_BRIDGE_CAMERA_ENABLED=false."""
    monkeypatch.setenv("SENSOR_BRIDGE_CAMERA_ENABLED", "false")
    config = BridgeConfig.from_env()
    assert config.camera_enabled is False


def test_config_from_env_unset_uses_defaults(monkeypatch):
    """Test that unset env vars leave defaults."""
    # Clear any relevant env vars to avoid test pollution
    for key in (
        "SENSOR_BRIDGE_CAMERA_PORT",
        "SENSOR_BRIDGE_LIDAR_PORT",
        "SENSOR_BRIDGE_CAMERA_ENABLED",
        "SENSOR_BRIDGE_LOG_LEVEL",
    ):
        monkeypatch.delenv(key, raising=False)
    config = BridgeConfig.from_env()
    assert config.camera_port == DEFAULT_CAMERA_PORT
    assert config.lidar_port == DEFAULT_LIDAR_PORT
    assert config.camera_enabled is True


def test_config_constructor_custom():
    """Test constructing BridgeConfig with custom values."""
    config = BridgeConfig(
        camera_port=8080,
        lidar_port=8081,
        camera_enabled=False,
        log_level=logging.WARNING,
    )
    assert config.camera_port == 8080
    assert config.lidar_port == 8081
    assert config.camera_enabled is False
    assert config.log_level == logging.WARNING
    assert config.sonar_port == DEFAULT_SONAR_PORT


def test_config_import_from_package():
    """Test that BridgeConfig can be imported from package root."""
    from mavsim_sensor_bridge import BridgeConfig as BC
    config = BC()
    assert config.camera_port == 8765
    assert config.lidar_port == 8766


def test_config_import_from_bridge():
    """Test that BridgeConfig can be imported from bridge (re-export)."""
    from mavsim_sensor_bridge.bridge import SensorBridge, BridgeConfig
    config = BridgeConfig()
    assert config.camera_port == 8765
    # SensorBridge accepts config
    bridge = SensorBridge(config=config)
    assert bridge.config.camera_port == 8765
