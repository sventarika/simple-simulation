import numpy as np

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.init_ks import init_ks
from vehiclemodels.init_st import init_st
from vehiclemodels.init_mb import init_mb

from commonroad.common.solution import VehicleModel, VehicleType
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics

from simulation_core.vehicle_state import VehicleState

from .vehicle_dynamics_object import VehicleDynamicsObject


class CommonRoadVehicleModel(VehicleDynamicsObject):
    """
    Lightweight vehicle dynamics simulator using the CommonRoad vehiclemodels.
    """

    def __init__(self,
                 initial_time_step: int,
                 initial_state: VehicleState,
                 dt: float,
                 vehicle_dynamics_model_name: str = "ks",
                 vehicle_type: int = 2
                 ) -> None:

        super().__init__(initial_time_step, initial_state, dt)

        # Vehicle Type
        allowed_vehicle_types = (2,)
        if vehicle_type not in allowed_vehicle_types:
            msg = f"Vehicle type '{vehicle_type}' not implemented. Choose from: {allowed_vehicle_types}"
            raise Exception(msg)

        if vehicle_type == 2:
            self._vehicle_parameters = parameters_vehicle2()
            vehicle_type_obj = VehicleType.BMW_320i

        # Dynamics Model
        allowed_vehicle_dynamics_model_names = ("ks", "st", "mb")
        if vehicle_dynamics_model_name not in allowed_vehicle_dynamics_model_names:
            msg = f"Model name '{vehicle_dynamics_model_name}' not implemented. Choose from: {allowed_vehicle_dynamics_model_names}"
            raise Exception(msg)

        initial_state_values = initial_state.values  # noqa: PD011

        if vehicle_dynamics_model_name == "ks":
            vehicle_model_obj = VehicleModel.KS
            initial_internal_state = init_ks(initial_state_values)
            internal_state_names = ("x", "y", "delta", "v", "theta")
        elif vehicle_dynamics_model_name == "st":
            vehicle_model_obj = VehicleModel.ST
            initial_internal_state = init_st(initial_state_values)
            internal_state_names = ("x", "y", "delta", "v", "theta", "theta_rate", "slip_angle")
        elif vehicle_dynamics_model_name == "mb":
            vehicle_model_obj = VehicleModel.MB
            initial_internal_state = init_mb(initial_state_values, self._vehicle_parameters)
            internal_state_names = ("x", "y", "delta", "v", "theta", "theta_rate", "slip_angle")

        self._vehicle_dynamics_model_name = vehicle_dynamics_model_name
        self._vehicle_dynamics = VehicleDynamics.from_model(vehicle_model_obj, vehicle_type_obj)

        self._internal_state_names = internal_state_names
        self._internal_state_list = [np.array(initial_internal_state)]

    @property
    def internal_state(self) -> np.ndarray:
        return self._internal_state_list[-1]

    @property
    def internal_state_list(self) -> np.ndarray:
        return np.array(self._internal_state_list)

    @property
    def length(self) -> float:
        return self._vehicle_parameters.l

    @property
    def width(self) -> float:
        return self._vehicle_parameters.w

    @property
    def max_delta_v(self) -> float:
        return self._vehicle_parameters.steering.v_max

    @property
    def max_a(self) -> float:
        return self._vehicle_parameters.longitudinal.a_max

    def step(self, delta_v: float, a: float) -> VehicleState:

        t0 = self._time_step * self._dt
        t1 = t0 + self._dt

        new_internal_state = self._simulate(t0, t1, delta_v, a)

        state_dict = dict(zip(self._internal_state_names, new_internal_state))
        new_state = VehicleState(**state_dict)
        self._state_list.append(new_state)

        self._time_step += 1

        return new_state

    def _simulate(self, t0: float, t1: float, delta_v: float, a: float) -> np.ndarray:

        cur_state = self._internal_state_list[-1]
        u = [delta_v, a]

        new_internal_state = self._vehicle_dynamics.forward_simulation(cur_state, u, t1 - t0, throw=True)

        # Display total acceleration
        # u2 = a
        # x4 = cur_state[3]
        # x5_dot = (cur_state[3] / (self._vehicle_parameters.a + self._vehicle_parameters.b)) * np.tan(cur_state[2])
        # total_acceleration = np.sqrt((u2 ** 2) + ((x4 * x5_dot) ** 2))
        # logger.debug("total_acceleration: {:.3f}", total_acceleration)

        self._input_list.append(u)
        self._internal_state_list.append(new_internal_state)

        return new_internal_state
