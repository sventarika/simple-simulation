from __future__ import annotations

from abc import abstractmethod
from copy import deepcopy


class Pilot:
    def __init__(self) -> None:
        # Monitoring
        self._state_list = []
        self._action_list = []

    @property
    def state_list(self) -> list:
        return deepcopy(self._state_list)

    @property
    def action_list(self) -> list:
        return deepcopy(self._action_list)

    def predict(self, observation: dict, **kwargs) -> tuple[float, float]:
        return self.step(observation, **kwargs)

    @abstractmethod
    def step(self, observation: dict, **kwargs) -> tuple[float, float]:
        """
        Make decision and decide on actions based on observation.
        """
        raise NotImplementedError
