"""
Bridge Configuration

Configuration dataclass for the SensorBridge with support for default values,
YAML file loading, and environment variable overrides.
"""

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Union

try:
    import yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


# Default ports per plan (Section 2.3)
DEFAULT_CAMERA_PORT = 8765
DEFAULT_LIDAR_PORT = 8766
DEFAULT_SONAR_PORT = 8767
DEFAULT_DEPTH_PORT = 8768
DEFAULT_AUXILIARY_PORT = 8769


@dataclass
class BridgeConfig:
    """
    Configuration for the SensorBridge.

    Attributes:
        camera_port: Port for camera server (default: 8765)
        lidar_port: Port for lidar server (default: 8766)
        sonar_port: Port for imaging sonar server (default: 8767)
        depth_port: Port for depth camera server (default: 8768)
        auxiliary_port: Port for auxiliary sensors (default: 8769)
        camera_enabled: Whether to enable camera server (default: True)
        lidar_enabled: Whether to enable lidar server (default: True)
        log_level: Logging level (default: logging.INFO)
    """
    camera_port: int = DEFAULT_CAMERA_PORT
    lidar_port: int = DEFAULT_LIDAR_PORT
    sonar_port: int = DEFAULT_SONAR_PORT
    depth_port: int = DEFAULT_DEPTH_PORT
    auxiliary_port: int = DEFAULT_AUXILIARY_PORT
    camera_enabled: bool = True
    lidar_enabled: bool = True
    log_level: int = logging.INFO

    @classmethod
    def from_yaml(cls, path: Union[str, Path]) -> "BridgeConfig":
        """
        Load configuration from a YAML file.

        Only keys that are valid BridgeConfig attributes are applied.
        Unknown keys are ignored. Missing keys use defaults.

        Args:
            path: Path to the YAML file (str or Path).

        Returns:
            BridgeConfig instance with values from the file.

        Raises:
            FileNotFoundError: If the file does not exist.
            ImportError: If PyYAML is not installed.
        """
        if not _HAS_YAML:
            raise ImportError("PyYAML is required for YAML config. Install with: pip install pyyaml")
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        if not data or not isinstance(data, dict):
            return cls()
        return cls._from_dict(data)

    @classmethod
    def from_env(cls) -> "BridgeConfig":
        """
        Load configuration from environment variables.

        Environment variables (optional overrides):
            SENSOR_BRIDGE_CAMERA_PORT: Camera server port (integer)
            SENSOR_BRIDGE_LIDAR_PORT: Lidar server port (integer)
            SENSOR_BRIDGE_SONAR_PORT: Sonar server port (integer)
            SENSOR_BRIDGE_DEPTH_PORT: Depth camera server port (integer)
            SENSOR_BRIDGE_AUXILIARY_PORT: Auxiliary server port (integer)
            SENSOR_BRIDGE_CAMERA_ENABLED: "true" or "false"
            SENSOR_BRIDGE_LIDAR_ENABLED: "true" or "false"
            SENSOR_BRIDGE_LOG_LEVEL: DEBUG, INFO, WARNING, ERROR

        Returns:
            BridgeConfig instance. Only set env vars override defaults.
        """
        config = cls()
        env = os.environ

        def _int(key: str, default: int) -> int:
            val = env.get(key)
            if val is None:
                return default
            try:
                return int(val)
            except ValueError:
                return default

        def _bool(key: str, default: bool) -> bool:
            val = env.get(key)
            if val is None:
                return default
            return val.strip().lower() in ("1", "true", "yes", "on")

        def _log_level(key: str, default: int) -> int:
            val = env.get(key)
            if val is None:
                return default
            return getattr(logging, val.strip().upper(), default)

        config.camera_port = _int("SENSOR_BRIDGE_CAMERA_PORT", config.camera_port)
        config.lidar_port = _int("SENSOR_BRIDGE_LIDAR_PORT", config.lidar_port)
        config.sonar_port = _int("SENSOR_BRIDGE_SONAR_PORT", config.sonar_port)
        config.depth_port = _int("SENSOR_BRIDGE_DEPTH_PORT", config.depth_port)
        config.auxiliary_port = _int("SENSOR_BRIDGE_AUXILIARY_PORT", config.auxiliary_port)
        config.camera_enabled = _bool("SENSOR_BRIDGE_CAMERA_ENABLED", config.camera_enabled)
        config.lidar_enabled = _bool("SENSOR_BRIDGE_LIDAR_ENABLED", config.lidar_enabled)
        config.log_level = _log_level("SENSOR_BRIDGE_LOG_LEVEL", config.log_level)
        return config

    @classmethod
    def _from_dict(cls, data: Dict[str, Any]) -> "BridgeConfig":
        """Build BridgeConfig from a dictionary (used by from_yaml)."""
        valid = {
            "camera_port", "lidar_port", "sonar_port", "depth_port", "auxiliary_port",
            "camera_enabled", "lidar_enabled", "log_level",
        }
        kwargs = {}
        for key, value in data.items():
            if key not in valid:
                continue
            if key == "log_level" and isinstance(value, str):
                value = getattr(logging, value.upper(), logging.INFO)
            kwargs[key] = value
        return cls(**kwargs)
