from __future__ import annotations

import do_mpc
import numpy as np

from casadi import if_else
from copy import deepcopy
from loguru import logger
from typing import TYPE_CHECKING

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2

if TYPE_CHECKING:
    import casadi.tools as castools


class MpcController:
    def __init__(
        self,
        dt: float,
        min_thw: float = 1.6,
        min_dhw_safety: float = 3.0,
        silent: bool = False,
        v_min: float = 0.0,
        v_max: float = 150 / 3.6,
        a_lim: float = 4.0,
        use_ma27: bool = False,
    ) -> None:
        """
        use_ma27: Whether to use MA27 as the linear solver. (Requires previous setup of HSL library.)
        """

        # Debug parameters
        self._log_predictions = True
        self._silent = silent
        self._use_ma27 = use_ma27

        # Parameters
        # General
        self.n_horizon = 15
        self.dt = dt

        # Model
        self._min_dhw_safety = min_thw  # m
        self._min_thw = min_dhw_safety  # s

        # Objective function
        # Possible extension: Weight decay over prediction horizon.
        self._cost_function_weights = {"cte": 1.0, "theta": 1.0, "v": 1.0, "thw": 10.0}
        # M term
        self.m_term_scale = 0.0  # static term in object function
        # Penalty for control inputs
        self.delta_v_penalty = 1e2  # Should not change delta too much in each time step
        self.acceleration_penalty = 1.0

        # Constraints
        # States
        self._delta_lim = np.deg2rad(45)
        self._v_min = v_min
        self._v_max = v_max
        # Inputs
        self._delta_v_lim = np.deg2rad(1)
        self._a_lim = a_lim

        # Init reference trajectory
        self._reference_values = {
            var_name: np.zeros((self.n_horizon,))
            for var_name in ("p_x", "p_y", "v", "theta", "initial_dhw", "lead_v_pred")
        }
        self._reference_values_set = (
            False  # whether or not the reference trajectory has been set yet
        )
        # Buffer
        self._reference_ref_line = None
        self._reference_curvilinear_cs = None

        # Setup routine
        self._prediction_buffer = []
        self._initial_guess_set = False
        self._vehicle_parameters = parameters_vehicle2()
        self._model = self._create_model()
        self._mpc = self._create_mpc()

    def _create_model(self) -> do_mpc.model.Model:
        """
        Kinematic single-track model
        """

        # Set the model type to 'continuous' (meaning the model represents a continuous-time system)
        model_type = "continuous"

        # Create a model object with the specified type
        model = do_mpc.model.Model(model_type)

        # Define the state variables of the model
        model.set_variable(var_type="_x", var_name="p_x", shape=(1, 1))  # x-position
        model.set_variable(var_type="_x", var_name="p_y", shape=(1, 1))  # y-position
        delta = model.set_variable(
            var_type="_x", var_name="delta", shape=(1, 1)
        )  # wheel steer angle
        v = model.set_variable(var_type="_x", var_name="v", shape=(1, 1))  # velocity
        theta = model.set_variable(
            var_type="_x", var_name="theta", shape=(1, 1)
        )  # heading
        model.set_variable(
            var_type="_x", var_name="dhw", shape=(1, 1)
        )  # dhw (distance to lead vehicle; -1 if no lead vehicle is present)

        # Define the control inputs of the model
        delta_v = model.set_variable(
            var_type="_u", var_name="delta_v", shape=(1, 1)
        )  # wheel steer rate
        a = model.set_variable(
            var_type="_u", var_name="a", shape=(1, 1)
        )  # longitudinal acceleration (in theta direction)

        # Define time-varying parameters of the model
        model.set_variable(var_type="_tvp", var_name="p_x_set")  # x-position setpoint
        model.set_variable(var_type="_tvp", var_name="p_y_set")  # y-position setpoint
        model.set_variable(
            var_type="_tvp", var_name="theta_set"
        )  # orientation setpoint
        model.set_variable(var_type="_tvp", var_name="v_set")  # velocity setpoint
        initial_dhw = model.set_variable(
            var_type="_tvp", var_name="initial_dhw"
        )  # dhw at this timestep (constant over the whole horizon)
        lead_v_pred = model.set_variable(
            var_type="_tvp", var_name="lead_v_pred"
        )  # absolute velocity of lead vehicle predicted over the horizon (-1 if there is no lead vehicle)

        # Define a parameter of the model (the wheel base of the vehicle)
        self._l_wb = (
            self._vehicle_parameters.a + self._vehicle_parameters.b
        )  # wheel base (distance between front and rear axle)

        # Set the right-hand side (RHS) of the differential equation for each state variable
        model.set_rhs(
            "p_x", v * np.cos(theta)
        )  # x-position is updated based on velocity and orientation
        model.set_rhs(
            "p_y", v * np.sin(theta)
        )  # y-position is updated based on velocity and orientation
        model.set_rhs(
            "delta", delta_v
        )  # wheel steer angle is updated based on wheel steer rate
        model.set_rhs("v", a)  # velocity is updated based on acceleration
        model.set_rhs(
            "theta", (v / self._l_wb) * np.tan(delta)
        )  # orientation is updated based on velocity, steering angle, and vehicle length
        model.set_rhs("dhw", if_else(initial_dhw == -1, -1, -(v - lead_v_pred)))

        # Finalize the model
        model.setup()

        return model

    def _create_mpc(self) -> do_mpc.controller.MPC:
        # Create an MPC controller object using the model created earlier
        mpc = do_mpc.controller.MPC(self._model)

        # Set up control parameters for the MPC controller
        setup_mpc = {
            "n_horizon": self.n_horizon,  # number of time steps in the prediction horizon
            "t_step": self.dt,  # time step size
            "n_robust": 1,  # number of uncertain parameters
            "store_full_solution": True,  # store the full solution of the optimization problem
            "nlpsol_opts": {},  # solver options for the optimization problem,
        }

        if self._use_ma27:
            setup_mpc["nlpsol_opts"]["ipopt.linear_solver"] = "MA27"

        if self._silent:
            setup_mpc["nlpsol_opts"]["ipopt.print_level"] = 0
            setup_mpc["nlpsol_opts"]["ipopt.sb"] = "yes"
            setup_mpc["nlpsol_opts"]["print_time"] = 0

        mpc.set_param(**setup_mpc)  # set the parameters for the MPC controller

        # Define the objective function for the optimization problem

        # -- Cross track error --
        # cross track error (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)
        theta_sin = np.sin(self._model.tvp["theta_set"])
        theta_cos = np.cos(self._model.tvp["theta_set"])
        dx = self._model.tvp["p_x_set"] - self._model.x["p_x"]
        dy = self._model.tvp["p_y_set"] - self._model.x["p_y"]
        lterm_cte = (theta_cos * dy - theta_sin * dx) ** 2

        # -- Heading error --
        # Taken from geometry.py (dseg)
        lterm_theta = (
            np.arctan2(
                np.sin(self._model.x["theta"] - self._model.tvp["theta_set"]),
                np.cos(self._model.x["theta"] - self._model.tvp["theta_set"]),
            )
        ) ** 2

        # -- Longitudinal error --
        # If no lead is present or it is far away, default value is "-1"
        no_lead_present = self._model.tvp["initial_dhw"] == -1
        # a) Velocity error
        lterm_v = if_else(
            no_lead_present, (self._model.x["v"] - self._model.tvp["v_set"]) ** 2, 0.0
        )
        # b) THW error
        # If no lead is present or it is far away, thw does not matter in the cost function
        thw_error_no_lead = 0.0
        # Set_thw if lead vehicle is present
        set_thw_with_lead_pow = (
            self._min_dhw_safety / self._model.x["v"]
        ) + self._min_thw
        # Actual thw at the moment
        current_thw_error = (
            set_thw_with_lead_pow - (self._model.x["dhw"] / self._model.x["v"])
        ) ** 2
        lterm_thw = self._cost_function_weights["thw"] * if_else(
            no_lead_present, thw_error_no_lead, current_thw_error
        )

        lterm = lterm_cte + lterm_theta + lterm_v + lterm_thw

        mterm = self.m_term_scale * lterm
        mpc.set_objective(
            lterm=lterm, mterm=mterm
        )  # set the objective function for the MPC controller

        # Set the control effort weighting factors
        mpc.set_rterm(
            delta_v=self.delta_v_penalty,  # weight on steering rate
            a=self.acceleration_penalty,  # weight on acceleration
        )

        # Set lower and upper bounds for the states and control inputs
        mpc.bounds[
            "lower", "_x", "delta"
        ] = -self._delta_lim  # lower bound on steering angle
        mpc.bounds["upper", "_x", "delta"] = (
            self._delta_lim
        )  # upper bound on steering angle
        mpc.bounds["lower", "_x", "v"] = self._v_min  # lower bound on velocity
        mpc.bounds["upper", "_x", "v"] = self._v_max  # upper bound on velocity

        mpc.bounds[
            "lower", "_u", "delta_v"
        ] = -self._delta_v_lim  # lower bound on steering rate
        mpc.bounds["upper", "_u", "delta_v"] = (
            self._delta_v_lim
        )  # upper bound on steering rate
        mpc.bounds["lower", "_u", "a"] = -self._a_lim  # lower bound on acceleration
        mpc.bounds["upper", "_u", "a"] = self._a_lim  # upper bound on acceleration

        # Get a template for the time-varying parameters
        tvp_template = mpc.get_tvp_template()

        # Define a function that updates the time-varying parameters to the current trajectory
        def tvp_fun(
            t_now: float,  # noqa: ARG001
        ) -> castools.structure3.SXStruct | castools.structure3.MXStruct:
            for k in range(self.n_horizon):
                tvp_template["_tvp", k, "p_x_set"] = self._reference_values["p_x"][
                    k
                ]  # x-position setpoint
                tvp_template["_tvp", k, "p_y_set"] = self._reference_values["p_y"][
                    k
                ]  # y-position setpoint
                tvp_template["_tvp", k, "v_set"] = self._reference_values["v"][
                    k
                ]  # velocity setpoint
                tvp_template["_tvp", k, "theta_set"] = self._reference_values["theta"][
                    k
                ]  # orientation setpoint
                tvp_template["_tvp", k, "initial_dhw"] = self._reference_values[
                    "initial_dhw"
                ][k]
                tvp_template["_tvp", k, "lead_v_pred"] = self._reference_values[
                    "lead_v_pred"
                ][k]

            return tvp_template

        # Set the function for updating the time-varying parameters
        mpc.set_tvp_fun(tvp_fun)

        # Finalize the MPC controller
        mpc.setup()

        return mpc

    def _log_prediction_info(self) -> None:
        """
        Track values of the cost function for plotting later on.
        """

        # First value is the current state
        predicted_state_values = {
            n: np.array(self._mpc.data.prediction(("_x", n))).reshape(
                self.n_horizon + 1
            )
            for n in self._mpc.x0.keys()  # noqa: SIM118
        }

        # Cross track error
        theta_sin = np.sin(self._reference_values["theta"])
        theta_cos = np.cos(self._reference_values["theta"])
        dx = self._reference_values["p_x"] - predicted_state_values["p_x"][1:]
        dy = self._reference_values["p_y"] - predicted_state_values["p_y"][1:]
        cte = np.abs(theta_cos * dy - theta_sin * dx)
        cte_pow = cte**2
        cte_pow_sum = np.sum(cte_pow)
        lterm_cte = self._cost_function_weights["cte"] * cte_pow_sum

        # Heading error
        predicted_theta = predicted_state_values["theta"][1:]
        ref_theta = self._reference_values["theta"]
        dtheta = np.arctan2(
            np.sin(predicted_theta - ref_theta), np.cos(predicted_theta - ref_theta)
        )
        dtheta_pow = dtheta**2
        dtheta_pow_sum = np.sum(dtheta_pow)
        lterm_theta = self._cost_function_weights["theta"] * dtheta_pow_sum

        # If no lead is present or it is far away, default value is "-1"
        no_lead_present = self._reference_values["initial_dhw"][0] == -1

        # Velocity error
        dv = self._reference_values["v"] - predicted_state_values["v"][1:]
        if no_lead_present:
            dv_pow = dv**2
            dv_pow_sum = np.sum(dv_pow)
            lterm_dv = self._cost_function_weights["v"] * dv_pow_sum
        else:
            lterm_dv = 0.0

        # Thw error
        cur_set_thw = (
            self._min_dhw_safety / predicted_state_values["v"][1:]
        ) + self._min_thw
        if no_lead_present:
            # If no lead is present or it is far away, thw does not matter in the cost function
            cur_thw_error = [0.0]
            lterm_thw = 0.0
        else:
            cur_thw = (
                predicted_state_values["dhw"][1:] / predicted_state_values["v"][1:]
            )
            cur_thw_error = cur_set_thw - cur_thw
            cur_thw_error_pow = cur_thw_error**2
            lterm_thw = self._cost_function_weights["thw"] * np.sum(cur_thw_error_pow)

        lterm = lterm_cte + lterm_theta + lterm_dv + lterm_thw

        if not self._silent:
            logger.debug("lterm_cte: {}", lterm_cte)
            logger.debug("lterm_theta: {}", lterm_theta)
            logger.debug("lterm_dv: {}", lterm_dv)
            logger.debug("lterm_thw: {}", lterm_thw)
            logger.debug("lterm: {}", lterm)

        buf = {
            "cte": cte,
            "dtheta": dtheta,
            "dv": dv,
            "cur_set_thw": cur_set_thw,
            "dthw": cur_thw_error,
            "lterm_cte": lterm_cte,
            "lterm_dtheta": lterm_theta,
            "lterm_dv": lterm_dv,
            "lterm_thw": lterm_thw,
            "lterm": lterm,
        }

        self._prediction_buffer.append(buf)

    @property
    def prediction_buffer(self) -> list:
        return deepcopy(self._prediction_buffer)

    @property
    def latest_prediction_buffer_entry(self) -> dict:
        return deepcopy(self._prediction_buffer[-1])

    def _update_reference_trajectory(
        self,
        ref_trajectory: np.ndarray,
        ref_trajectory_theta: np.ndarray,
        ref_velocity: np.ndarray,
        cur_dhw: float,
        cur_lead_v: float,
    ) -> None:
        self._reference_values["p_x"] = ref_trajectory[:, 0]
        self._reference_values["p_y"] = ref_trajectory[:, 1]
        self._reference_values["v"] = ref_velocity * np.ones((self.n_horizon,))
        self._reference_values["theta"] = ref_trajectory_theta

        self._reference_values["initial_dhw"] = cur_dhw * np.ones((self.n_horizon,))
        self._reference_values["lead_v_pred"] = cur_lead_v * np.ones((self.n_horizon,))

        self._reference_values_set = True

    def make_step(
        self,
        current_state: list,
        ref_trajectory: np.ndarray,
        ref_trajectory_theta: np.ndarray,
        ref_velocity: np.ndarray,
        cur_dhw: float = -1,
        cur_lead_v: float = -1,
    ) -> tuple[float, float]:
        self._update_reference_trajectory(
            ref_trajectory, ref_trajectory_theta, ref_velocity, cur_dhw, cur_lead_v
        )

        if not self._reference_values_set:
            msg = "Reference trajectory has to be updated (by calling update_reference_trajectory()) before make_step() can be called."
            raise Exception(msg)

        if not self._initial_guess_set:
            self._mpc.x0 = np.array(current_state).reshape(
                -1, 1
            )  # set the initial state of the system
            self._mpc.set_initial_guess()  # set the initial guess for the optimization problem
            self._initial_guess_set = (
                True  # set the flag to indicate that the initial guess has been set
            )

        u = self._mpc.make_step(np.array(current_state).reshape(-1, 1))

        if self._log_predictions:
            self._log_prediction_info()

        self._reference_values_set = False

        return u[0, 0], u[1, 0]
