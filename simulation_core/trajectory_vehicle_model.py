from __future__ import annotations

import numpy as np

from typing import TYPE_CHECKING
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2

from .vehicle_dynamics_object import VehicleDynamicsObject

if TYPE_CHECKING:
    from .vehicle_state import VehicleState


class TrajectoryVehicleModel(VehicleDynamicsObject):
    def __init__(
        self,
        initial_time_step: int,
        initial_state: VehicleState,
        dt: float,
        state_list_after_initial_step: list[VehicleState],
        vehicle_type: int = 2,
    ) -> None:
        super().__init__(initial_time_step, initial_state, dt)

        # Vehicle type
        allowed_vehicle_types = (2,)
        if vehicle_type not in allowed_vehicle_types:
            msg = f"Vehicle type '{vehicle_type}' not implemented. Choose from: {allowed_vehicle_types}"
            raise Exception(msg)

        if vehicle_type == 2:
            self._vehicle_parameters = parameters_vehicle2()

        # Directly fill complete state list
        self._internal_state_list = [initial_state]
        self._internal_state_list.extend(state_list_after_initial_step)
        self._input_list = [np.array([np.nan, np.nan])] * len(self._state_list)

    @property
    def length(self) -> float:
        return self._vehicle_parameters.l

    @property
    def width(self) -> float:
        return self._vehicle_parameters.w

    def step(self, *args, **kwargs) -> VehicleState:  # noqa: ARG002
        self._time_step += 1
        new_state = self._internal_state_list[self._time_step - self._initial_time_step]

        self._state_list.append(new_state)

        return new_state
