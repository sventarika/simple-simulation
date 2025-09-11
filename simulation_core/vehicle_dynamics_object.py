import numpy as np

from abc import abstractmethod
from loguru import logger

from .vehicle_state import VehicleState


class VehicleDynamicsObject:
    """
    Super class of all vehicle dynamic simulators.
    """

    def __init__(self, initial_time_step: int, initial_state: VehicleState, dt: float) -> None:

        self._initial_time_step = initial_time_step
        self._time_step = initial_time_step
        self._dt = dt

        self._state_list = [initial_state]
        self._input_list = [np.nan, np.nan]

        logger.info("Initial state: {}", self.state)

    @abstractmethod
    def step(self, *args, **kwargs) -> VehicleState:
        """Add next VehicleState to _state_list based on inputs."""

    @property
    def time_step(self) -> int:
        return self._time_step

    @property
    def observation(self) -> VehicleState:
        observable_state = VehicleState(**self.state.asdict(filter_non_available=True), length=self.length, width=self.width)
        return observable_state

    @property
    def state(self) -> VehicleState:
        return self._state_list[-1]

    @property
    @abstractmethod
    def length(self) -> float:
        """Length of the vehicle."""

    @property
    @abstractmethod
    def width(self) -> float:
        """Width of the vehicle."""
