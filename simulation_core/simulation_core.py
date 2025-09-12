from __future__ import annotations

import cv2
import matplotlib as mpl
import numpy as np

from copy import deepcopy
from loguru import logger
from pathlib import Path
from typing import TYPE_CHECKING

from commonroad_dc import pycrcc
from commonroad_dc.boundary import boundary
from commonroad_dc.collision.trajectory_queries.trajectory_queries import (
    obb_enclosure_polygons_static,
)

from simple_scenario import Scenario

from .situation_renderer import SituationRenderer
from .termination_status import TerminationStatus
from .trajectory_vehicle_model import TrajectoryVehicleModel
from .commonroad_vehicle_model import CommonRoadVehicleModel
from .vehicle_state import VehicleState

if TYPE_CHECKING:
    from .vehicle_dynamics_object import VehicleDynamicsObject


class SimulationCore:
    """
    Lightweight simulation using CommonRoad vehicle models.
    """

    def __init__(
        self,
        scenario_or_config: str | Path | dict | Scenario,
        live_plot: bool = False,
        write_video_path: str | None = None,
        write_video_fps: float = 40.0,
        i_job: int | None = None,
        silent: bool = False,
    ) -> None:
        self._verbose = not silent

        if self._verbose:
            logger.info("Create {}", self.__class__.__name__)

        self._i_job = i_job

        self._scenario_or_config = scenario_or_config

        # Lazy loading to allow initialization of many SimulationCores before starting the simulation.
        self._scenario_name = None
        self._scenario = None
        self._planning_problem = None
        self._lanelet_network_wrapper = None

        self._time_step = None
        self._termination_status = None

        # Vehicle Dynamic Simulators
        self._vehicle_dynamics_objects = None

        # Collision objects
        self._road_inclusion_polygon_group = None
        self._road_boundary_collision_object = None
        self._goal_region_collision_object = None

        self._ego_vehicle_collision_object = None
        self._object_vehicle_collision_objects = None

        # Visualization
        self._live_plot = live_plot
        self._write_video_path = write_video_path
        self._video_writer = None
        self._video_fps = write_video_fps
        self._video_shape = None
        self._cr_renderer = SituationRenderer()

        # Cache
        self._action_list = []
        self._obs_list = []

    @property
    def action_list(self) -> list:
        return deepcopy(self._action_list)

    @property
    def observation_list(self) -> list:
        return deepcopy(self._obs_list)

    def reset(self, visualize: bool = False) -> dict:
        if self._verbose:
            logger.info("{}Reset {}", self._get_job_str(), self.__class__.__name__)

        cv2.destroyAllWindows()

        if self._video_writer:
            if self._verbose:
                logger.info("Finish video")
            self._video_writer.release()

        # Load simple scenario (lazy loading)
        simple_scenario = Scenario.from_x(self._scenario_or_config)

        # Extract common road (CR) scenario (that is built up automatically inside of the simple scenario), because the simulation needs a CR scenario
        self._scenario_name = simple_scenario.id
        self._scenario = simple_scenario.get_cr_interface().scenario
        self._planning_problem = simple_scenario.get_cr_interface().planning_problem
        self._lanelet_network_wrapper = (
            simple_scenario.get_cr_interface().lanelet_network_wrapper
        )

        self._termination_status = TerminationStatus()

        self._time_step = 0
        self._dt = self._scenario.dt

        # Init all vehicle simulators
        self._vehicle_dynamics_objects = self._create_vehicle_dynamics_objects()

        obs = self._observe()
        self._obs_list.append(obs)

        self._update_collision_objects()

        if visualize:
            self._show()

        return obs

    def step(self, actions: dict) -> tuple[np.ndarray, float, bool, dict]:
        if self._verbose:
            logger.info(
                "{}Step {} (time step: {} -> {})",
                self._get_job_str(),
                self.__class__.__name__,
                self._time_step,
                self._time_step + 1,
            )

        self._action_list.append(actions)

        # Actual update to states
        for do_id, action in actions.items():
            self._vehicle_dynamics_objects[do_id].step(action[0], action[1])

        self._time_step += 1

        obs = self._observe()
        self._obs_list.append(obs)

        reward = None

        self._update_collision_objects()
        done = self._check_termination()
        info = self._create_info_dict()

        self._show()

        return obs, reward, done, info

    def _create_vehicle_dynamics_objects(
        self,
    ) -> dict[str | int, VehicleDynamicsObject]:
        if self._verbose:
            logger.info("Create vehicle dynamics objects")

        vehicle_dynamics_objects = {}

        # Ego vehicle
        if self._verbose:
            logger.info("Create vehicle dynamics object for ego")

        ego_state = self._planning_problem.initial_state
        initial_ego_state = VehicleState(
            x=ego_state.position[0],
            y=ego_state.position[1],
            delta=0.0,
            v=ego_state.velocity,
            theta=ego_state.orientation,
        )
        ego_dynamics_object = CommonRoadVehicleModel(
            self._time_step, initial_ego_state, self._dt
        )
        vehicle_dynamics_objects["ego"] = ego_dynamics_object

        # Object vehicles
        if self._verbose:
            logger.info("Create vehicle dynamics objects for object vehicles")

        for dobj in self._scenario.dynamic_obstacles:
            if self._verbose:
                logger.info(
                    "Create vehicle dynamics objects for object vehicle: {}",
                    dobj.obstacle_id,
                )

            initial_object_vehicle_state = VehicleState(
                x=dobj.initial_state.position[0],
                y=dobj.initial_state.position[1],
                delta=0.0,
                v=dobj.initial_state.velocity,
                theta=dobj.initial_state.orientation,
            )

            # TODO(vater): Use some kind of config to assign VehicleDynamicModels instead of hardcoding (keep Pilots in mind as well)
            # https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/simple-simulation/simple-simulation/-/issues/1
            dynamics_model = "traj"  # "cr"

            if dynamics_model == "cr":
                object_vehicle_dynamics_object = CommonRoadVehicleModel(
                    self._time_step, initial_object_vehicle_state, self._dt
                )
            elif dynamics_model == "traj":
                other_states = [
                    VehicleState(
                        x=s.position[0],
                        y=s.position[1],
                        delta=0.0,
                        v=s.velocity,
                        theta=s.orientation,
                    )
                    for s in dobj.prediction.trajectory.state_list
                ]

                object_vehicle_dynamics_object = TrajectoryVehicleModel(
                    self._time_step,
                    initial_object_vehicle_state,
                    self._dt,
                    state_list_after_initial_step=other_states,
                )
            else:
                raise Exception

            vehicle_dynamics_objects[dobj.obstacle_id] = object_vehicle_dynamics_object

        return vehicle_dynamics_objects

    def _observe(self) -> dict:
        lanelet_map = None
        if self._time_step == 0:
            lanelet_map = self._scenario.lanelet_network

        dynamic_objects_observation = {}
        for do_id, vehicle_dynamics_object in self._vehicle_dynamics_objects.items():
            dynamic_objects_observation[do_id] = (
                vehicle_dynamics_object.observation.asdict(filter_non_available=True)
            )

        obs = {
            "general": {"dt": self._dt},
            "road": {"lanelet_map": lanelet_map},
            "dynamic_objects": dynamic_objects_observation,
        }

        return obs

    def _update_collision_objects(self) -> None:
        # Road and goal
        if self._time_step == 0:
            self._road_inclusion_polygon_group = boundary.create_road_polygons(
                self._scenario,
                method="lane_polygons",
                buffer=1,
                resample=1,
                triangulate=False,
            )
            _, self._road_boundary_collision_object = (
                boundary.create_road_boundary_obstacle(self._scenario)
            )

            if len(self._planning_problem.goal.state_list) != 1:
                msg = "Only one goal region allowed."
                raise Exception(msg)

            goal_state_rect = self._planning_problem.goal.state_list[0].position
            self._goal_region_collision_object = pycrcc.RectOBB(
                goal_state_rect.length / 2,
                goal_state_rect.width / 2,
                goal_state_rect.orientation,
                goal_state_rect.center[0],
                goal_state_rect.center[1],
            )

        # Collision objects
        self._object_vehicle_collision_objects = {}

        for do_id, vehicle_dynamics_obj in self._vehicle_dynamics_objects.items():
            obj_observation = vehicle_dynamics_obj.observation
            collision_obj = pycrcc.RectOBB(
                obj_observation.length / 2,
                obj_observation.width / 2,
                obj_observation.theta,
                obj_observation.x,
                obj_observation.y,
            )
            if do_id == "ego":
                self._ego_vehicle_collision_object = collision_obj
            else:
                self._object_vehicle_collision_objects[do_id] = collision_obj

    def _check_termination(self) -> bool:
        done = False

        # Offroad check
        if self._time_step == 0:
            self._termination_status.is_offroad = not obb_enclosure_polygons_static(
                self._road_inclusion_polygon_group, self._ego_vehicle_collision_object
            )

        self._termination_status.is_offroad = (
            self._termination_status.is_offroad
            or self._road_boundary_collision_object.collide(
                self._ego_vehicle_collision_object
            )
        )

        # Collision check
        for do_id, collision_obj in self._object_vehicle_collision_objects.items():
            is_collision_with_this_obj = self._ego_vehicle_collision_object.collide(
                collision_obj
            )
            if is_collision_with_this_obj:
                if self._verbose:
                    logger.info("Collision with object vehicle {}", do_id)
                self._termination_status.collision_obj_id = do_id
            self._termination_status.is_collision = (
                self._termination_status.is_collision or is_collision_with_this_obj
            )

        # Timeout check; reset is already time_step 0, if there are 200 time_steps in total, 199 is the last one.
        max_time = (
            max(
                [
                    state.time_step.end
                    for state in self._planning_problem.goal.state_list
                ]
            )
            - 1
        )

        if self._time_step >= max_time:
            self._termination_status.is_time_out = True

        # Goal reached check
        self._termination_status.is_goal_reached = (
            self._ego_vehicle_collision_object.collide(
                self._goal_region_collision_object
            )
        )

        # Standstill check
        self._termination_status.is_standstill = self._vehicle_dynamics_objects[
            "ego"
        ].state.v < (1 / 3.6)

        done = self._termination_status.is_terminated()

        return done

    def _create_info_dict(self) -> dict:
        # Calculate DHW, THW, TTC
        dhw = np.inf
        thw = np.inf
        ttc = np.inf

        lanelet_wrapper = self._lanelet_network_wrapper
        cur_do_obs = self._obs_list[-1]["dynamic_objects"]
        ego_obs = cur_do_obs["ego"]

        other_obj_obs = {
            do_id: obj for do_id, obj in cur_do_obs.items() if do_id != "ego"
        }

        # Find lead vehicle
        ego_llt_id = lanelet_wrapper.find_lanelet_id_by_position(
            ego_obs["x"], ego_obs["y"]
        )
        surrounding_vehicles = lanelet_wrapper.find_surrounding_vehicles(
            ego_obs["x"], ego_obs["y"], other_obj_obs
        )
        lead_vehicle = surrounding_vehicles.lead

        if lead_vehicle:
            ego_s, _ = lanelet_wrapper.from_cart_to_llt_frenet(
                ego_llt_id, ego_obs["x"], ego_obs["y"]
            )
            lead_s, _ = lanelet_wrapper.from_cart_to_llt_frenet(
                ego_llt_id, lead_vehicle["x"], lead_vehicle["y"]
            )

            dhw = np.round(lead_s - ego_s, 2)
            thw = np.round(dhw / ego_obs["v"], 4)

            dv = ego_obs["v"] - lead_vehicle["v"]

            if dv > 0:
                ttc = np.round(dhw / dv, 4)

        info = {
            "time_step": self._time_step,
            "termination_status": self._termination_status.asdict(),
            "dhw": dhw,
            "thw": thw,
            "ttc": ttc,
        }

        return info

    def _get_job_str(self) -> str:
        job_str = ""
        if self._i_job is not None:
            job_str = f"Job {self._i_job:05d}: "
        return job_str

    def _show(self) -> None:  # noqa: PLR0912
        debug_image = False

        if not self._live_plot and not self._write_video_path and not debug_image:
            return

        id_fontsize = 2

        self._scenario.lanelet_network.draw(
            self._cr_renderer, {"lanelet": {"fill_lanelet": False}}
        )

        # Draw goal position
        self._goal_region_collision_object.draw(
            self._cr_renderer,
            draw_params={"facecolor": "red", "edgecolor": "red", "zorder": 20},
        )

        # Draw ego vehicle
        self._ego_vehicle_collision_object.draw(
            self._cr_renderer,
            draw_params={"facecolor": "blue", "edgecolor": "blue", "zorder": 30},
        )

        # Draw object vehicles
        for obj in self._object_vehicle_collision_objects.values():
            obj.draw(
                self._cr_renderer,
                draw_params={"facecolor": "green", "edgecolor": "green", "zorder": 30},
            )

        # Draw ids
        for do_id, obj in self._vehicle_dynamics_objects.items():
            draw_params = {"facecolor": "green", "edgecolor": "green", "zorder": 30}

            if do_id == "ego":
                draw_params["facecolor"] = "blue"
                center = self._ego_vehicle_collision_object.center()
            else:
                center = self._object_vehicle_collision_objects[do_id].center()

            id_label = mpl.text.Text(
                center[0],
                center[1],
                f"{do_id} | {obj.state.v * 3.6:.0f} km/h",
                fontsize=id_fontsize,
                zorder=31,
            )
            self._cr_renderer.static_artists.append(id_label)

        # To draw the road boundaries used to calculate off-road
        # self._road_inclusion_polygon_group.draw(self._cr_renderer)  # noqa: ERA001
        # self._road_boundary_collision_object.draw(self._cr_renderer)  # noqa: ERA001

        # Set plot limits; TODO: Adapt to road width
        visible_range_ego_x = 300
        visible_range_ego_y = 25

        ego_state = self._vehicle_dynamics_objects["ego"].state
        ego_x = ego_state.x
        ego_y = ego_state.y
        ego_theta = ego_state.theta

        local_xmin = ego_x - visible_range_ego_x * np.cos(ego_theta)
        local_xmax = ego_x + visible_range_ego_x * np.cos(ego_theta)

        local_ymin = ego_y - visible_range_ego_y * np.sin(ego_theta + np.pi / 2)
        local_ymax = ego_y + visible_range_ego_y * np.sin(ego_theta + np.pi / 2)

        plot_lims = [local_xmin, local_xmax, local_ymin, local_ymax]

        fsize = (9, 9)

        if debug_image:
            filename = "test.jpg"
        else:
            filename = None

        render_frame = self._cr_renderer.render(
            show=False,
            keep_static_artists=False,
            mode="rgb_array",
            figsize=fsize,
            plot_limits=plot_lims,
            filename=filename,
        )

        render_frame = render_frame.astype(np.float32)

        # Add frame number + termination reason
        termination_reason = ""
        if self._termination_status.is_terminated():
            termination_reason = next(
                name
                for name, status in self._termination_status.asdict().items()
                if status
            )

        cv2.putText(
            render_frame,
            f"{self._time_step} ({self._time_step * self._dt:.1f}s) {termination_reason}",
            (0, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            color=[0, 0, 0],
            thickness=1,
            lineType=cv2.LINE_AA,
        )

        render_frames = [render_frame]

        if render_frames is None:
            cv2.destroyAllWindows()

            if self._video_writer:
                self._video_writer.release()

            return

        render_frame = render_frames[0]

        if self._live_plot:
            cv2.imshow(
                self._scenario_name,
                cv2.cvtColor(render_frame, cv2.COLOR_RGB2BGR).astype(np.uint8),
            )

            # Press Q on keyboard to  exit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return

        if self._write_video_path:
            if self._video_writer is None:
                self._video_shape = render_frame.shape[:2]

                if self._write_video_path.is_dir():
                    video_dir = Path(self._write_video_path)
                    video_file = video_dir / f"{self._scenario_name}.mp4"
                else:
                    video_file = Path(self._write_video_path)
                    video_dir = video_file.parent
                if not video_dir.exists():
                    video_dir.mkdir(parents=True)

                codec = "avc1"  # generated videos can be watched in a web browser or in VSCode, but not in default opencv installation (need to build opencv from sources); if you use default opencv installation, use "mp4v" instead, because with "avc1" no video will be created at all

                # Delete old video
                video_file.unlink(missing_ok=True)

                while not video_file.exists():

                    self._video_writer = cv2.VideoWriter(
                        str(video_file),
                        cv2.VideoWriter_fourcc(*codec),  # avc1 for webvideos
                        self._video_fps,
                        (int(self._video_shape[1]), int(self._video_shape[0])),
                        True,
                    )

                    if codec == "mp4v" and not video_file.exists():
                        msg = "Tried to use codec 'mp4v' after 'avc1', but both did not work."
                        raise RuntimeError(msg)

                    codec = "mp4v"

            if self._video_shape != render_frame.shape[:2]:
                raise Exception

            render_frame_video = cv2.cvtColor(render_frame, cv2.COLOR_RGBA2BGR).astype(
                np.uint8
            )

            self._video_writer.write(render_frame_video)
