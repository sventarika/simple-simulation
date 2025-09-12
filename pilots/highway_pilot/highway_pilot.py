from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy
from loguru import logger
from typing import TYPE_CHECKING

from simple_scenario import LaneletNetworkWrapper

from ..mpc_controller import MpcController
from ..pilot import Pilot

if TYPE_CHECKING:
    from pathlib import Path
    from simple_scenario.lanelet_network_wrapper.neighbour_lanes import NeighbourLanes
    from simple_scenario.lanelet_network_wrapper.surrounding_vehicles import (
        SurroundingVehicles,
    )


class HighwayPilot(Pilot):
    def __init__(
        self, dt: float = 0.1, ego_id: str | int = "ego", silent: bool = True
    ) -> None:
        super().__init__()

        # General settings
        self._dt = dt
        self._ego_id = ego_id
        self._silent = silent

        # ACC
        # Timegap control
        self._min_timegap = 1.6  # s
        self._min_dhw_safety = 3  # m
        self._timegap_control_active = True  # Timegap control is active from the beginning. This is safer, because there could already be a close lead vehicle. If not, it wil be deactivated right away.
        self._should_activate_timegap_control = False
        self._prev_lead_v = None

        # AEB
        self._aeb_active = False
        self._aeb_counter = 0.0
        self._aeb_drac = 0.0
        self._aeb_t_brake_stage_1 = 0.4
        self._aeb_a_brake_stage_1 = -4
        self._aeb_a_brake_stage_2 = -6

        # LCA
        self._waiting_period_after_LC = 5  # s
        self._LC_active = False
        self._time_in_current_LC = None
        self._LC_target_llt_id = None
        self._LC_ref_trajectory = None
        self._LC_ref_trajectory_theta = None
        self._time_since_LC = None

        # Subfunctions
        self._lanelet_wrapper = None
        self._mpc_controller = MpcController(self._dt, silent=True)

        # Monitoring
        self._additional_monitoring_values = {}

    @property
    def additional_monitoring_values(self) -> dict:
        return deepcopy(self._additional_monitoring_values)

    def step(self, observation: dict, **kwargs) -> tuple[float, float]:  # noqa: ARG002, PLR0912
        # Assumption: Map will never change in one simulation run. Thus, the lanelet wrapper is only set once.
        if self._lanelet_wrapper is None:
            self._lanelet_wrapper = LaneletNetworkWrapper(
                observation["road"]["lanelet_map"]
            )

        # Ego state
        ego_object = observation["dynamic_objects"][self._ego_id]

        ego_x = ego_object["x"]
        ego_y = ego_object["y"]
        ego_delta = ego_object["delta"]
        ego_v = ego_object["v"]
        ego_theta = ego_object["theta"]
        ego_length = ego_object["length"]

        speed_limit = self._lanelet_wrapper.get_speed_limit_by_pos(ego_x, ego_y) / 3.6

        ego_ref_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(ego_x, ego_y)
        ego_lanelet_id = self._lanelet_wrapper.find_lanelet_id_by_position(ego_x, ego_y)
        cur_min_timegap = self._min_dhw_safety / ego_v + self._min_timegap

        if self._action_list:
            ego_a = self._action_list[-1][1]
        else:
            ego_a = 0.0

        other_dynamic_objects = {
            do_id: do_obj
            for do_id, do_obj in observation["dynamic_objects"].items()
            if do_id is not self._ego_id
        }

        # -- Longitudinal control

        # ACC (adaptive cruise control)
        # ACC has to modes:
        # a) Velocity control: No lead vehicle, keep ref_velocity
        # b) Timegap control: Lead vehicle, keep set timegap (=thw), defined by self._min_timegap and self._min_dhw_safety
        # Only one of them is activate at a time.
        # If cur_dhw and cur_lead_v are other than -1, the timegap control will be used. If they are -1, the velocity control will be used.

        # Velocity control
        ref_velocity = speed_limit

        # Timegap control (=THW control)
        surrounding_vehicles = self._lanelet_wrapper.find_surrounding_vehicles(
            ego_x, ego_y, other_dynamic_objects
        )

        # Avoid undertaking
        if (
            surrounding_vehicles.left_lead
            and surrounding_vehicles.left_lead["v"] < ref_velocity
        ):
            ref_velocity = surrounding_vehicles.left_lead["v"]

        if surrounding_vehicles.lead is None:
            cur_dhw = -1.0
            cur_thw = -1.0
            cur_lead_v = -1.0
            cur_lead_a = 0.0
            lead_vehicle_ref_s = -1.0
            lead_vehicle_length = 0
            self._timegap_control_active = False
        else:
            lead_vehicle = surrounding_vehicles.lead
            lead_vehicle_ref_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(
                lead_vehicle["x"], lead_vehicle["y"]
            )

            lead_vehicle_length = lead_vehicle["length"]

            cur_dhw = (
                lead_vehicle_ref_s
                - lead_vehicle_length / 2
                - ego_length / 2
                - ego_ref_s
            )
            cur_thw = cur_dhw / ego_v

            cur_lead_v = lead_vehicle["v"]

            if self._prev_lead_v is not None:
                cur_lead_a = (cur_lead_v - self._prev_lead_v) / self._dt
            else:
                cur_lead_a = 0.0

            if self._timegap_control_active:
                # If timegap control is already active, it should only be deactivated if the lead vehicle is driver faster than the ref_velocity of the ego
                # AND if either the current thw is higher than the minimum required timegap OR the ego velocity is higher than the ref_velocity
                should_deactivate_timegap_control = cur_lead_v >= ref_velocity and (
                    cur_thw > cur_min_timegap or ego_v > ref_velocity
                )
                self._should_activate_timegap_control = (
                    not should_deactivate_timegap_control
                )
            else:
                # If timegap control is not active, it should only be activated if the THW is lower than the minimum required THW AND the ego velocity is higher than the ref_velocity
                # The case that the THW is lower than the minimum req. THW AND the ego is driving slower will never happen (only in the first time step, but there the timegap control is always already active)
                self._should_activate_timegap_control = (
                    cur_thw < cur_min_timegap and ego_v > ref_velocity
                )

            if self._should_activate_timegap_control:
                self._timegap_control_active = True
            else:
                cur_dhw = -1.0
                cur_thw = -1.0
                cur_lead_v = -1.0
                cur_lead_a = 0.0
                self._timegap_control_active = False

            self._prev_lead_v = cur_lead_v

        # AEB
        lead_vehicle_present = (
            surrounding_vehicles.lead is not None and self._timegap_control_active
        )
        self._check_aeb(
            lead_vehicle_present,
            ego_ref_s + ego_length / 2,
            lead_vehicle_ref_s - lead_vehicle_length / 2,
            ego_v,
            cur_lead_v,
            ego_a,
            cur_lead_a,
        )

        # -- Lateral control --
        n_horizon = self._mpc_controller.n_horizon

        # LCA (lane change assist)
        if self._aeb_active:
            self._LC_active = False

        elif self._LC_active:
            self._time_in_current_LC += self._dt

            # Check where we are on the reference trajectory
            ego_target_llt_s, _ = self._lanelet_wrapper.from_cart_to_llt_frenet(
                self._LC_target_llt_id, ego_x, ego_y
            )
            lc_trajectory_target_llt_frenet = (
                self._lanelet_wrapper.point_array_from_cart_to_llt_frenet(
                    self._LC_target_llt_id, self._LC_ref_trajectory
                )
            )

            remaining_lc_ref_trajectory_target_llt_frenet = (
                lc_trajectory_target_llt_frenet[
                    lc_trajectory_target_llt_frenet[:, 0] >= ego_target_llt_s
                ]
            )
            remaining_lc_ref_trajectory = self._LC_ref_trajectory[
                lc_trajectory_target_llt_frenet[:, 0] >= ego_target_llt_s
            ]
            remaining_lc_ref_trajectory_theta = self._LC_ref_trajectory_theta[
                lc_trajectory_target_llt_frenet[:, 0] >= ego_target_llt_s
            ]

            n_remaining_lc_ref_trajectory_points = len(remaining_lc_ref_trajectory)

            if n_remaining_lc_ref_trajectory_points >= n_horizon:
                ref_trajectory = remaining_lc_ref_trajectory[:n_horizon]
                ref_trajectory_theta = remaining_lc_ref_trajectory_theta[:n_horizon]

            elif n_remaining_lc_ref_trajectory_points > 0:
                # Append some points from target llt centerline at the end if needed
                n_additional_points = n_horizon - n_remaining_lc_ref_trajectory_points
                s_dist_per_step = ref_velocity * self._dt
                additional_points_s = lc_trajectory_target_llt_frenet[
                    -1, 0
                ] + np.linspace(
                    s_dist_per_step,
                    (n_additional_points + 1) * s_dist_per_step,
                    n_additional_points,
                )

                ref_trajectory_frenet = np.zeros((n_horizon, 2))
                ref_trajectory_theta = np.zeros((n_horizon,))

                ref_trajectory_frenet[:n_remaining_lc_ref_trajectory_points] = (
                    remaining_lc_ref_trajectory_target_llt_frenet
                )
                ref_trajectory_frenet[n_remaining_lc_ref_trajectory_points:, 0] = (
                    additional_points_s
                )

                ref_trajectory = (
                    self._lanelet_wrapper.point_array_from_llt_frenet_to_cart(
                        self._LC_target_llt_id, ref_trajectory_frenet
                    )
                )

                ref_trajectory_theta[:n_remaining_lc_ref_trajectory_points] = (
                    remaining_lc_ref_trajectory_theta
                )
                ref_trajectory_theta[n_remaining_lc_ref_trajectory_points:] = (
                    self._lanelet_wrapper.heading_along_llt(
                        self._LC_target_llt_id, additional_points_s
                    )
                )

            else:
                self._LC_active = False
                self._time_since_LC = -self._dt

        else:
            neighbour_lanes = self._lanelet_wrapper.find_available_neighbour_lanes(
                ego_lanelet_id
            )
            lane_change_direction = self._check_lane_change(
                ego_x,
                ego_y,
                ego_length,
                ego_v,
                ref_velocity,
                cur_dhw,
                cur_lead_v,
                neighbour_lanes,
                surrounding_vehicles,
            )

            if lane_change_direction != 0 and (
                self._time_since_LC is None
                or self._time_since_LC > self._waiting_period_after_LC
            ):
                lanelet_id_lc1 = ego_lanelet_id + lane_change_direction
                lc_duration = 4

                self._LC_ref_trajectory = self._generate_lc_trajectory(
                    ego_ref_s, ego_v, ego_lanelet_id, lanelet_id_lc1, lc_duration
                )
                self._LC_ref_trajectory_theta = self._calculate_theta_of_trajectory(
                    self._LC_ref_trajectory
                )

                ref_trajectory = self._LC_ref_trajectory[:n_horizon]
                ref_trajectory_theta = self._LC_ref_trajectory_theta[:n_horizon]

                self._LC_active = True
                self._LC_target_llt_id = lanelet_id_lc1
                self._time_in_current_LC = 0.0

        if not self._LC_active:
            if self._time_since_LC is not None:
                self._time_since_LC += self._dt

            # LKA (lane keep assist)
            # In LKA, the ego vehicle will follow the centerline of the lanelet it is currently in.
            # Create a ref_trajectory along the ego_lanelet's centerline.
            ego_cur_s, _ = self._lanelet_wrapper.from_cart_to_llt_frenet(
                ego_lanelet_id, ego_x, ego_y
            )
            s_dist_per_step = ref_velocity * self._dt
            ref_trajectory_frenet = np.zeros((n_horizon, 2))
            ref_trajectory_frenet[:, 0] = ego_cur_s + np.linspace(
                s_dist_per_step, (n_horizon + 1) * s_dist_per_step, n_horizon
            )
            ref_trajectory = self._lanelet_wrapper.point_array_from_llt_frenet_to_cart(
                ego_lanelet_id, ref_trajectory_frenet
            )
            ref_trajectory_theta = self._lanelet_wrapper.heading_along_llt(
                ego_lanelet_id, ref_trajectory_frenet[:, 0]
            )

        # -- MPC --
        current_state = [ego_x, ego_y, ego_delta, ego_v, ego_theta, cur_dhw]

        delta_v, a = self._mpc_controller.make_step(
            current_state,
            ref_trajectory,
            ref_trajectory_theta,
            ref_velocity,
            cur_dhw,
            cur_lead_v,
        )

        if self._aeb_active:
            self._LC_active = False
            if self._aeb_counter <= self._aeb_t_brake_stage_1:
                a = max(self._aeb_a_brake_stage_1, self._aeb_drac)
            elif cur_lead_v > 0:
                a = max(
                    self._aeb_a_brake_stage_2, 1.1 * self._aeb_drac
                )  # now it is serious, brake harder than needed
            else:
                a = self._aeb_a_brake_stage_2

        # -- Monitoring --
        self._state_list.append(current_state)
        self._action_list.append([delta_v, a])

        additional_values_to_monitor = {
            "ref_trajectory": ref_trajectory,
            "ref_trajectory_theta": ref_trajectory_theta,
            "ref_velocity": ref_velocity,
            "timegap_control_active": self._timegap_control_active,
            "cur_lead_v": cur_lead_v,
            "cur_min_timegap": cur_min_timegap,
            "cur_thw": cur_thw,
            "aeb_active": self._aeb_active,
        }
        mpc_prediction_buffer = self._mpc_controller.latest_prediction_buffer_entry
        for k in ("cte", "dtheta", "dv", "dthw"):
            additional_values_to_monitor[k] = mpc_prediction_buffer[k][0]

        self._add_to_additional_monitoring_values(additional_values_to_monitor)

        return delta_v, a

    def _check_aeb(
        self,
        lead_vehicle_present: bool,
        ego_front_bumper_s: float,
        lead_vehicle_rear_bumper_s: float,
        ego_v: float,
        lead_v: float,
        ego_a: float = 0.0,  # noqa: ARG002
        lead_a: float = 0.0,
    ) -> None:
        if lead_vehicle_present:
            # The threshold for AEB activation depends on the current speed of ego and whether the lead vehicle is moving
            v_AEB = (
                np.array(
                    [
                        0,
                        10,
                        20,
                        30,
                        40,
                        50,
                        60,
                        70,
                        80,
                        90,
                        100,
                        110,
                        120,
                        130,
                        140,
                        150,
                        250,
                    ]
                )
                / 3.6
            )
            if lead_v > 1:
                # Front vehicle Moving
                threshold_AEB = np.array(
                    [
                        0.85,
                        0.85,
                        0.9,
                        0.95,
                        1.1,
                        1.2,
                        1.3,
                        1.4,
                        1.5,
                        1.5,
                        1.5,
                        1.5,
                        1.5,
                        1.5,
                        1.5,
                        1.5,
                        1.5,
                    ]
                )
            else:
                # Front vehicle stand still
                threshold_AEB = np.array(
                    [
                        0.75,
                        0.75,
                        0.8,
                        0.95,
                        1.1,
                        1.2,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                        1.4,
                    ]
                )

            cur_ttc_threshold = np.interp(ego_v, v_AEB, threshold_AEB)

            drac_threshold = -1

            # DRAC
            drac, ttc = self._drac(
                ego_front_bumper_s,
                lead_vehicle_rear_bumper_s,
                ego_v,
                lead_v,
                acceleration_based=True,
                rear_a0=0.0,
                lead_a0=lead_a,
            )
            dv = ego_v - lead_v

            if np.isnan(ttc):
                ttc = 999

            # AEB control
            if self._aeb_active:
                deactivate_aeb = (
                    ttc > cur_ttc_threshold
                    and drac > drac_threshold
                    and dv < 0.0
                    and lead_a > 0.0
                    and lead_v > 0.0
                )
                if deactivate_aeb:
                    self._aeb_active = False
                    self._aeb_counter = 0.0
                    self._aeb_drac = 0.0
                else:
                    self._aeb_active = True
                    self._aeb_counter += self._dt
                    self._aeb_drac = drac
            elif ttc < cur_ttc_threshold or drac < self._aeb_a_brake_stage_1:
                self._aeb_active = True
                self._aeb_counter += self._dt
                self._aeb_drac = drac
            else:
                self._aeb_active = False
                self._aeb_counter = 0.0
                self._aeb_drac = 0.0
        else:
            self._aeb_active = False
            self._aeb_counter = 0.0
            self._aeb_drac = 0.0

    def _check_lane_change(  # noqa: PLR0912, PLR0911
        self,
        ego_x: float,
        ego_y: float,
        ego_length: float,
        ego_v: float,
        ref_velocity: float,
        dhw: float,
        lead_v: float,
        neighbour_lane_availability: NeighbourLanes,
        surrounding_vehicles: SurroundingVehicles,
    ) -> int:
        # PARAMS
        t_limit_LC = 5  # time to wait between LCs
        v_limit_trigger_LC = (
            10.8 / 3.6
        )  # driving below ref velocity -> try to do left LC
        v_limit_close_to_set_velocity = 3 / 3.6

        v_limit_LC = 60 / 3.6

        THW_front_vehicle_limit = 4

        THW_front_right_lane_free = 3
        dx_min_right_lane_free = 70

        THW_front_left_lane_free = 3
        dx_min_left_lane_free = 70

        # This is for disallowing LCs at the beginning of the simulation. Needed?

        dx_limit_side = 10
        THW_limit_side = 1
        a_req_limit = -2

        dx_res = 5

        if self._time_since_LC:
            self._time_since_LC += self._dt

            # If LC was done recently, do not do another one
            if self._time_since_LC < t_limit_LC:
                return 0

        # If velocity is too small, no lane change shall be performed

        # Lane change check
        delta_ref_v = ref_velocity - ego_v

        # Check whether a lane change should be considered from the ego lanes perspective

        lead_THW = 20
        if ego_v > 2 and dhw > 0:
            lead_THW = dhw / ego_v

        consider_lane_change_to = 0
        if ego_v < v_limit_LC:
            if not self._silent:
                logger.debug("Speed is lower than minimum speed for LC. Stay in lane.")
            return 0

        if delta_ref_v > v_limit_trigger_LC and lead_THW < THW_front_vehicle_limit:
            consider_lane_change_to = 1
            if not self._silent:
                logger.debug(
                    "Speed is lower than set speed and lead vehicle quite close, consider doing a lane change to left."
                )
        elif delta_ref_v < v_limit_close_to_set_velocity:
            consider_lane_change_to = -1
            if not self._silent:
                logger.debug(
                    "Set velocity is reached. Check if a lane change to the right lane makes sense."
                )

        if consider_lane_change_to == 0:
            # If not lane change is meaningful right now, do not check other lanes
            return 0

        ego_ref_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(ego_x, ego_y)

        if consider_lane_change_to == 1:
            # Check situation on left lane

            # There is no left lane
            if not neighbour_lane_availability.left_lane_available:
                if not self._silent:
                    logger.debug("No left lane")
                return 0

            left_lead_dhw = 999

            if surrounding_vehicles.left_lead:
                left_lead_vehicle = surrounding_vehicles.left_lead
                left_lead_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(
                    left_lead_vehicle["x"], left_lead_vehicle["y"]
                )

                left_lead_dhw = (
                    left_lead_s
                    - surrounding_vehicles.left_lead["length"] / 2
                    - ego_ref_s
                    - ego_length / 2
                )

            left_rear_dhw = 999
            left_rear_dv = 999

            if surrounding_vehicles.left_rear:
                left_rear_vehicle = surrounding_vehicles.left_rear
                left_rear_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(
                    left_rear_vehicle["x"], left_rear_vehicle["y"]
                )

                left_rear_dhw = (
                    ego_ref_s
                    - ego_length / 2
                    - left_rear_s
                    - surrounding_vehicles.left_rear["length"] / 2
                )
                left_rear_v = left_rear_vehicle["v"]
                left_rear_dv = ego_v - left_rear_v

            # Required decelerations for the rear vehicles
            a_req_left_rear = -9.81
            if left_rear_dhw > dx_res:
                if left_rear_dv > 0:
                    a_req_left_rear = 0
                else:
                    a_req_left_rear, _ = self._drac(
                        left_rear_s, ego_ref_s, left_rear_v, ego_v
                    )

            # THW
            left_lead_THW = 20
            left_rear_THW = 20

            if ego_v > 2:
                if surrounding_vehicles.left_lead:
                    left_lead_THW = left_lead_dhw / ego_v
                if surrounding_vehicles.left_rear:
                    left_rear_THW = left_rear_dhw / left_rear_v

            # Go through criteria that prohibit a lane change

            # Left rear or lead vehicle are really close
            if left_lead_dhw < dx_limit_side or left_rear_dhw < dx_limit_side:
                if not self._silent:
                    logger.debug("Adjacent lane vehicles too close")
                return 0

            # THW to left rear or lead vehicle is too small
            if left_lead_THW < THW_limit_side or left_rear_THW < THW_limit_side:
                if not self._silent:
                    logger.debug("Adjacent lane vehicles' TTC too small")
                return 0

            # Required deceleration to avoid a crash for left rear is too high
            if surrounding_vehicles.left_rear and a_req_left_rear < a_req_limit:
                if not self._silent:
                    logger.debug(
                        "Adjacent lane rear vehicles' required deceleration too high"
                    )
                return 0

            # Left lead vehicle is too close, such that left lane cannot be considered free
            if (
                surrounding_vehicles.left_lead
                and left_lead_THW < THW_front_left_lane_free
                and left_lead_dhw < dx_min_left_lane_free
            ):
                if not self._silent:
                    logger.debug(
                        f"Adjacent lane lead vehicle not far enough away (THW: {left_lead_THW:.1f}s < {THW_front_left_lane_free:.1f}s, DHW: {left_lead_dhw:.1f}m < {dx_min_left_lane_free:.1f}m)"
                    )
                return 0

            # If non of the above are true, do LC to left
            return 1

        elif consider_lane_change_to == -1:  # noqa: RET505
            if not neighbour_lane_availability.right_lane_available:
                if not self._silent:
                    logger.debug("No right lane")
                return 0

            # Avoid undertaking
            # If there is a lead vehicle that is driving below ref_velocity, do not perform a LC to the right.
            if (
                surrounding_vehicles.lead
                and self._timegap_control_active
                and lead_v < ref_velocity
            ):
                if not self._silent:
                    logger.debug(
                        "Timegap control and lead vehicle has lower velocity than ego. Stay behind it to avoid undertaking."
                    )
                return 0

            # Check if right lane is "free"

            right_lead_dhw = 999

            if surrounding_vehicles.right_lead:
                right_lead_vehicle = surrounding_vehicles.right_lead
                right_lead_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(
                    right_lead_vehicle["x"], right_lead_vehicle["y"]
                )

                right_lead_dhw = (
                    right_lead_s
                    - right_lead_vehicle["length"] / 2
                    - ego_ref_s
                    - ego_length / 2
                )

            right_rear_dhw = 999
            right_rear_dv = 999

            if surrounding_vehicles.right_rear:
                right_rear_vehicle = surrounding_vehicles.right_rear
                right_rear_s, _ = self._lanelet_wrapper.from_cart_to_ref_frenet(
                    right_rear_vehicle["x"], right_rear_vehicle["y"]
                )

                right_rear_dhw = ego_ref_s - right_rear_s
                right_rear_v = right_rear_vehicle["v"]
                right_rear_dv = ego_v - right_rear_v

            # Required decelerations for the rear vehicles
            a_req_right_rear = -9.81
            if right_rear_dhw > dx_res:
                if right_rear_dv > 0:
                    a_req_right_rear = 0
                else:
                    a_req_right_rear, _ = self._drac(
                        right_rear_s, ego_ref_s, right_rear_v, ego_v
                    )

            # THW
            right_lead_THW = 20
            right_rear_THW = 20

            if ego_v > 2:
                if surrounding_vehicles.right_lead:
                    right_lead_THW = right_lead_dhw / ego_v
                if surrounding_vehicles.right_rear:
                    right_rear_THW = right_rear_dhw / right_rear_v

            # Go through criteria that prohibit a lane change

            # Right rear or lead vehicle are really close
            if right_lead_dhw < dx_limit_side or right_rear_dhw < dx_limit_side:
                if not self._silent:
                    logger.debug("Adjacent lane vehicles too close")
                return 0

            # THW to right rear or lead vehicle is too small
            if right_lead_THW < THW_limit_side or right_rear_THW < THW_limit_side:
                if not self._silent:
                    logger.debug("Adjacent lane vehicles' TTC too small")
                return 0

            # Required deceleration to avoid a crash for right rear is too high
            if surrounding_vehicles.right_rear and a_req_right_rear < a_req_limit:
                if not self._silent:
                    logger.debug(
                        "Adjacent lane rear vehicles' required deceleration too high"
                    )
                return 0

            # Right lead vehicle is too close, such that left lane cannot be considered free
            if (
                surrounding_vehicles.right_lead
                and right_lead_THW < THW_front_right_lane_free
                and right_lead_dhw < dx_min_right_lane_free
            ):
                if not self._silent:
                    logger.debug(
                        f"Adjacent lane lead vehicle not far enough away (THW: {right_lead_THW:.1f}s < {THW_front_right_lane_free:.1f}s, DHW: {right_lead_dhw:.1f}m < {dx_min_right_lane_free:.1f}m)"
                    )
                return 0

            # If non of the above are true, do LC to left
            return -1
        return None

    @staticmethod
    def _drac(
        rear_s0: float,
        lead_s0: float,
        rear_v0: float,
        lead_v0: float,
        acceleration_based: bool = False,
        rear_a0: float = 0.0,
        lead_a0: float = 0.0,
    ) -> tuple[float, float]:
        """
        Deceleration to avoid crash (rear decelerates to avoid crash with lead).
        """

        if acceleration_based and lead_a0 < -1:
            # TTC with acceleration
            if lead_a0 >= rear_a0:
                ttca = np.nan
            else:
                # % Time until collision (like classical TTC, but assuming constant acceleration instead of constant speed)
                ds0 = rear_s0 - lead_s0
                dv0 = rear_v0 - lead_v0
                da0 = rear_a0 - lead_a0

                ttca = np.sqrt(
                    0.25 * (((2 * dv0) / da0) ** 2) - ((2 * ds0) / da0)
                ) - 0.5 * ((2 * dv0) / da0)

            if np.isnan(ttca):
                drac = 0.0
            else:
                drac = (lead_v0 + lead_a0 * ttca - rear_v0) / ttca

            return drac, ttca

        else:  # noqa: RET505
            # Time to collision (no acceleration involved)
            if lead_v0 >= rear_v0:
                tc = np.nan
            else:
                tc = (lead_s0 - rear_s0) / (rear_v0 - lead_v0)

            # Acceleration such that rear vehicle can decelerate below (or equal to) the lead velocity at the point of crash
            if np.isnan(tc):
                a_req = 0.0
            else:
                a_req = (
                    lead_v0 - rear_v0
                ) / tc  # Upper limit. This barely avoids the crash!

            return a_req, tc

    def _generate_lc_trajectory(
        self,
        s_lc0: float,
        v0: float,
        llt_id_lc0: int,
        llt_id_lc1: int,
        lc_duration: float,
        a: float = 0.0,
    ) -> np.ndarray:
        dt = self._dt

        # Keep velocity, constant acceleration over lc_duration
        v1 = v0 + a * lc_duration

        # s position after lc
        s_lc1 = s_lc0 + ((v0 + v1) / 2) * lc_duration

        # Lateral positions in ref_frame
        x_lc0, y_lc0 = self._lanelet_wrapper.from_llt_frenet_to_cart(
            llt_id_lc0, s_lc0, 0.0
        )
        x_lc1, y_lc1 = self._lanelet_wrapper.from_llt_frenet_to_cart(
            llt_id_lc1, s_lc1, 0.0
        )

        _, t_lc0 = self._lanelet_wrapper.from_cart_to_ref_frenet(x_lc0, y_lc0)
        _, t_lc1 = self._lanelet_wrapper.from_cart_to_ref_frenet(x_lc1, y_lc1)

        # 5th order polynomial
        d = lc_duration
        p = np.array(
            [
                [0**5, 0**4, 0**3, 0**2, 0**1, 1],  # lateral start position
                [d**5, d**4, d**3, d**2, d**1, 1],  # lateral end positioin
                [0, 0, 0, 0, 1, 0],  # lateral start velocity
                [5 * d**4, 4 * d**3, 3 * d**2, 2 * d, 1, 0],  # lateral end velocity
                [0, 0, 0, 2, 0, 0],  # lateral start acceleration
                [20 * d**3, 12 * d**2, 6 * d, 2, 0, 0],
            ]
        )  # lateral end acceleration
        pv = np.linalg.solve(p, [t_lc0, t_lc1, 0.0, 0.0, 0, 0])

        trajectory_pts = []
        time_steps = np.arange(0, lc_duration + dt, dt)

        for t in time_steps:
            delta_s = s_lc0 + v0 * t + 0.5 * a * t * t
            delta_t = (
                pv[0] * t**5
                + pv[1] * t**4
                + pv[2] * t**3
                + pv[3] * t**2
                + pv[4] * t**1
                + pv[5]
            )
            trajectory_pts.append([delta_s, delta_t])

        trajectory_pts = np.array(trajectory_pts)

        # From frenet to cart
        trajectory_pts_cart = self._lanelet_wrapper.point_array_from_ref_frenet_to_cart(
            trajectory_pts
        )

        return trajectory_pts_cart

    @staticmethod
    def _calculate_theta_of_trajectory(trajectory: np.ndarray) -> np.ndarray:
        diff = np.diff(trajectory, axis=0)

        theta_raw = np.arctan2(diff[:, 1], diff[:, 0])

        # Prepend first theta
        theta = np.concatenate((np.array(theta_raw[0]).reshape((1)), theta_raw))

        return theta

    def _add_to_additional_monitoring_values(self, current_values: dict) -> None:
        for k, v in current_values.items():
            if k not in self._additional_monitoring_values:
                self._additional_monitoring_values[k] = []

            self._additional_monitoring_values[k].append(v)

    def create_monitor_plots(self, plot_dir: Path, scenario_name: str) -> None:
        # -- Read monitored values --

        # ADF monitoring
        states = np.array(self.state_list)
        actions = np.array(self.action_list)
        additional_values = self.additional_monitoring_values

        # Simulation monitoring

        # Derive values
        ref_trajectory = np.array(additional_values["ref_trajectory"])[:, 0, :]
        ref_trajectory_theta = np.array(additional_values["ref_trajectory_theta"])[:, 0]
        ref_velocity = np.array(additional_values["ref_velocity"])

        lead_v = np.array(additional_values["cur_lead_v"])
        timegap_control_active = np.array(additional_values["timegap_control_active"])
        cur_thw = np.array(additional_values["cur_thw"])
        cur_min_timegap = np.array(additional_values["cur_min_timegap"])

        aeb_active = np.array(additional_values["aeb_active"])

        cte = np.array(additional_values["cte"])
        dtheta = np.array(additional_values["dtheta"])
        dv = np.array(additional_values["dv"])
        dthw = np.array(additional_values["dthw"])

        ego_x = states[:, 0]
        ego_y = states[:, 1]
        ego_delta = states[:, 2]
        ego_v = states[:, 3]
        ego_theta = states[:, 4]

        delta_v = actions[:, 0]
        a = actions[:, 1]

        # Masked values
        cur_thw_during_timegap_control = np.copy(cur_thw)
        cur_thw_during_timegap_control[np.logical_not(timegap_control_active)] = np.nan

        ego_v_during_timegap_control = np.copy(ego_v)
        ego_v_during_timegap_control[np.logical_not(timegap_control_active)] = np.nan

        a_during_timegap_control = np.copy(a)
        a_during_timegap_control[np.logical_not(timegap_control_active)] = np.nan

        dv_during_velocity_control = np.copy(dv)
        dv_during_velocity_control[timegap_control_active] = 0.0

        dthw_during_timegap_control = np.copy(dthw)
        dthw_during_timegap_control[np.logical_not(timegap_control_active)] = 0.0

        # -- Plots --
        img_ext = "png"

        logger.info("Create plots in {}", plot_dir.absolute())

        # -- Reference plot --
        logger.info("Ref line plot")
        fig, axs = plt.subplots(3)
        fig.set_size_inches((9, 12))
        fig.suptitle("Reference")

        ax = axs[0]
        ax.plot(
            ref_trajectory[:, 0],
            ref_trajectory[:, 1],
            "r--",
            label="Ref line in cart. cs",
        )
        ax.set(title="Trajectory", xlabel="x in m", ylabel="y in m")
        ax.grid()
        ax.legend()

        ax = axs[1]
        ax.plot(ref_velocity, "r--", label="Target velocity")
        ax.set(title="Velocity", xlabel="Timestep", ylabel="m/s")
        ax_twin = ax.twinx()
        ax_twin.plot(3.6 * ref_velocity, alpha=0.0)
        ax_twin.set(ylabel="km/h")
        ax.grid()
        ax.legend()

        ax = axs[2]
        ax.plot(ref_trajectory_theta, "r--", label="Theta of ref_line")
        ax.set(title="Heading", xlabel="Timestep", ylabel="rad")
        ax_twin = ax.twinx()
        ax_twin.plot(np.rad2deg(ref_trajectory_theta), alpha=0.0)
        ax_twin.set(ylabel="deg")
        ax.grid()
        ax.legend()

        fig.tight_layout()
        fig.savefig(plot_dir / f"{scenario_name}_ref_line.{img_ext}")
        plt.close(fig)

        # -- Longitudinal control --
        logger.info("Longitudinal control plot")
        fig, axs = plt.subplots(6)
        fig.set_size_inches((9, 12))

        ax = axs[0]
        ax.set_title("THW")
        ax.plot(cur_thw, "c--", label="actual THW")
        ax.plot(cur_thw_during_timegap_control, "b-", label="masked THW")
        ax.plot(cur_min_timegap, "r--", label="Target THW")
        ax.set(ylabel="s")
        ax.legend()
        ax.grid()

        ax = axs[1]
        ax.set_title("Velocity")
        ax.plot(ego_v, "c-", label="v")
        ax.plot(ego_v_during_timegap_control, "b-", label="v (thw ctrl)")
        ax.plot(ref_velocity, "r--", label="ref_velocity")
        ax.plot(lead_v, "g--", label="lead_v")
        ax.set(ylabel="m/s")
        ax.legend()
        ax.grid()

        ax = axs[2]
        ax.plot(timegap_control_active, "b-", label="Timegap control active")
        ax.plot(2 * aeb_active.astype(int), "g-", label="AEB active")
        ax.legend()
        ax.grid()

        ax = axs[3]
        ax.set_title("THW Error")
        ax.plot(dthw_during_timegap_control, "r-", label="thw error in s")
        ax.set(ylabel="s")
        ax.legend()
        ax.grid()

        ax = axs[4]
        ax.set_title("V Error")
        ax.plot(dv_during_velocity_control, "m-", label="v error")
        ax.set(ylabel="m/s")
        ax.legend()
        ax.grid()

        ax = axs[5]
        ax.plot(a, "r-", label="a")
        ax.plot(a_during_timegap_control, "b-", label="masked_a")
        ax.set(ylabel="m/s^2")
        ax.legend()
        ax.grid()

        axs[-1].set(xlabel="Timestep")

        fig.tight_layout()
        fig.savefig(plot_dir / f"{scenario_name}_thw.{img_ext}")
        plt.close(fig)

        # -- Cross track error --
        logger.info("Cross track error plot")
        fig, axs = plt.subplots(3)
        fig.set_size_inches((9, 12))

        ax = axs[0]
        ax.plot(ref_trajectory[:, 0], ref_trajectory[:, 1], "r--", label="ref_line")
        ax.plot(ego_x, ego_y, "b-", label="Trajectory in cart. cs")
        ax.set(title="Position", xlabel="x in m", ylabel="y in m")
        ax.legend()
        ax.grid()

        ax = axs[1]
        ax.plot(ego_x, cte, "r-", label="Cross track error")
        ax.set(xlabel="x in m", ylabel="m")
        ax.legend()
        ax.grid()

        ax = axs[2]
        ax.plot(cte, "r-", label="Cross track error")
        ax.set(xlabel="Timestep", ylabel="m")
        ax.legend()
        ax.grid()

        fig.tight_layout()
        fig.savefig(plot_dir / f"{scenario_name}_cross_track_error.{img_ext}")
        plt.close(fig)

        # -- Heading error --
        logger.info("Heading error plot")
        fig, axs = plt.subplots(4)
        fig.set_size_inches((9, 14))
        ax = axs[0]
        ax.plot(ref_trajectory_theta, "r--", label="Ref Theta")
        ax.plot(ego_theta, "b-", label="Theta")
        ax.set(title="Heading", xlabel="Timestep", ylabel="rad")
        ax_twin = ax.twinx()
        ax_twin.plot(np.rad2deg(ref_trajectory_theta), alpha=0)
        ax_twin.set(ylabel="deg")
        ax.legend()
        ax.grid()

        ax = axs[1]
        ax.plot(dtheta, "r-", label="Theta error")
        ax.set(title="Theta error", xlabel="Timestep", ylabel="rad")
        ax_twin = ax.twinx()
        ax_twin.plot(np.rad2deg(dtheta), alpha=0)
        ax_twin.set(ylabel="deg")
        ax.grid()
        ax.legend()

        ax = axs[2]
        ax.plot(ego_delta, "b-", label="delta")
        ax.set(title="Delta", xlabel="Timestep", ylabel="rad")
        ax_twin = ax.twinx()
        ax_twin.plot(np.rad2deg(ego_delta), alpha=0)
        ax_twin.set(ylabel="deg")
        ax.legend()
        ax.grid()

        ax = axs[3]
        ax.plot(delta_v, "r-", label="delta_v")
        ax.set(title="Delta_v", xlabel="Timestep", ylabel="rad/s")
        ax_twin = ax.twinx()
        ax_twin.plot(np.rad2deg(delta_v), alpha=0)
        ax_twin.set(ylabel="deg/s")
        ax.legend()
        ax.grid()

        fig.tight_layout()
        fig.savefig(plot_dir / f"{scenario_name}_heading.{img_ext}")
        plt.close(fig)
