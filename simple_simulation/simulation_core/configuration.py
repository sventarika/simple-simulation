from __future__ import annotations

from asyncio.log import logger
import orjson
from typing import Any
from pathlib import Path

# Default configuration values
DEFAULT_CONFIG = {
    "scenario": "",
    "simulation": {
        "real_time_factor": 1.0,
        "monitor_pilots": True,
        "save_video": False,
        "save_result": "simulation_data",
        "i_job": None,
        "log_level": "INFO",
    },
    "termination_criteria": {
        "max_duration": 1000.0,
        "collision": True,
        "timeout": True,
        "off_road": True,
        "goal_reached": True,
        "standstill": True,
    },
    "ego": {"id": "ego", "pilot": "highway_pilot", "dynamics_model": "cr"},
    "objects": {
        "default_pilot": "highway_pilot",
        "default_dynamics_model": "traj",
        "specific_objects": {},
    },
}


class SimulationCoreConfiguration:
    def __init__(
        self,
        configuration_file_path: str | Path | None = None,
    ) -> None:
        self._config = self._load_configuration(configuration_file_path)

    def _load_configuration(
        self, configuration_file_path: str | Path | None
    ) -> dict[str, Any]:
        if configuration_file_path is None:
            logger.info(
                "No configuration file path provided. Using default configuration."
            )
            return DEFAULT_CONFIG

        with Path.open(configuration_file_path) as f:
            loaded_config = orjson.loads(f.read())

        # Merge with defaults (loaded values take precedence)
        config = DEFAULT_CONFIG.copy()
        self._deep_update(config, loaded_config)

        # Basic validation: Check required top-level keys and types
        required_keys = [
            "scenario",
            "simulation",
            "termination_criteria",
            "ego",
            "objects",
        ]
        for key in required_keys:
            if key not in config:
                msg = f"Missing required config key: {key}"
                raise ValueError(msg)
            if not isinstance(config[key], (str, dict)):
                msg = f"Invalid type for config key '{key}': expected str or dict"
                raise TypeError(msg)

        return config

    def _deep_update(self, base: dict[str, Any], update: dict[str, Any]) -> None:
        """Recursively update base dict with update dict."""
        for key, value in update.items():
            if isinstance(value, dict) and key in base and isinstance(base[key], dict):
                self._deep_update(base[key], value)
            else:
                base[key] = value

    @property
    def config(self) -> dict[str, Any]:
        return self._config

    @property
    def scenario(self) -> str:
        return self._config.get("scenario", "")

    @property
    def simulation(self) -> dict[str, Any]:
        return self._config.get("simulation", {})

    @property
    def termination_criteria(self) -> dict[str, Any]:
        return self._config.get("termination_criteria", {})

    @property
    def ego(self) -> dict[str, Any]:
        return self._config.get("ego", {})

    @property
    def objects(self) -> dict[str, Any]:
        return self._config.get("objects", {})
