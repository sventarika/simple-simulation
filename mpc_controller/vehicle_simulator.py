import numpy as np

from loguru import logger

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.init_ks import init_ks
from vehiclemodels.init_st import init_st
from vehiclemodels.init_mb import init_mb

from commonroad.common.solution import VehicleModel, VehicleType
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics


class VehicleSimulator:
    """
    Lightweight vehicle dynamics simulator.
    """

    def __init__(self,
                 model_name: str = "ks",
                 vehicle_type: int = 2,
                 x0: float = 0.0,
                 y0: float = 0.0,
                 delta0: float = 0.0,
                 v0: float = 27.78,
                 theta0: float = 0.0,
                 theta_rate0: float = 0.0,
                 slip_angle0: float = 0.0
                 ) -> None:

        allowed_models = ("ks", "st", "mb")
        if model_name not in allowed_models:
            msg = f"Model name '{model_name}' not implemented. Choose from: {allowed_models}"
            raise Exception(msg)

        allowed_vehicle_types = (2,)
        if vehicle_type not in allowed_vehicle_types:
            msg = f"Vehicle type '{vehicle_type}' not implemented. Choose from: {allowed_vehicle_types}"
            raise Exception(msg)

        # Model parameters
        self._initial_p_x = x0
        self._initial_p_y = y0
        self._initial_delta = delta0
        self._initial_v = v0
        self._initial_theta = theta0
        self._initial_theta_rate = theta_rate0
        self._initial_slip_angle = slip_angle0

        initial_state_values = [
            self._initial_p_x,
            self._initial_p_y,
            self._initial_delta,
            self._initial_v,
            self._initial_theta,
            self._initial_theta_rate,
            self._initial_slip_angle
        ]

        if vehicle_type == 2:
            self._vehicle_parameters = parameters_vehicle2()
            vehicle_type_obj = VehicleType.BMW_320i

        if model_name == "ks":
            vehicle_model_obj = VehicleModel.KS
            self._initial_state = init_ks(initial_state_values)
        elif model_name == "st":
            vehicle_model_obj = VehicleModel.ST
            self._initial_state = init_st(initial_state_values)
        elif model_name == "mb":
            vehicle_model_obj = VehicleModel.MB
            self._initial_state = init_mb(initial_state_values, self._vehicle_parameters)

        self._vehicle_dynamic = VehicleDynamics.from_model(vehicle_model_obj, vehicle_type_obj)

        self._state_list = [self._initial_state]
        self._input_list = [[np.nan, np.nan]]

    @property
    def cur_state(self) -> list:
        return self._state_list[-1]

    @property
    def state_list(self) -> np.ndarray:
        return np.array(self._state_list)

    @property
    def input_list(self) -> np.ndarray:
        return np.array(self._input_list)

    def simulate(self, t0: float, t1: float, delta_v: float, a: float) -> list:

        cur_state = self._state_list[-1]
        u = [delta_v, a]

        new_state = self._vehicle_dynamic.forward_simulation(cur_state, u, t1 - t0, throw=True)

        # Display total acceleration
        u2 = a
        x4 = cur_state[3]
        x5_dot = (cur_state[3] / (self._vehicle_parameters.a + self._vehicle_parameters.b)) * np.tan(cur_state[2])
        total_acceleration = np.sqrt((u2 ** 2) + ((x4 * x5_dot) ** 2))
        logger.debug("total_acceleration: {:.3f}", total_acceleration)

        self._input_list.append(u)
        self._state_list.append(new_state)

        return new_state
