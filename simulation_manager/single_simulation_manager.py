from __future__ import annotations

import numpy as np
import orjson

from loguru import logger
from time import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path

from simulation_core import SimulationCore
from simple_scenario import Scenario
from pilots import DummyPilot, HighwayPilot


class SingleSimulationManager:
    def __init__(
        self,
        scenario_file_or_config: str | Path | dict | Scenario,
        result_dir: str | None = None,
        i_job: int | None = None,
        live_plot: bool = False,
        save_video: bool = False,
        save_result: bool = False,
        monitor_pilots: bool = False,
    ) -> None:
        self._scenario = Scenario.from_x(scenario_file_or_config)

        self._i_job = i_job
        self._live_plot = live_plot
        self._save_result = save_result
        self._save_video = save_video
        self._monitor_pilots = monitor_pilots

        # Result dir handling
        self._result_dir = result_dir
        self._video_dir = None
        self._monitor_plot_dir = None

        if result_dir is None:
            if self._save_video:
                msg = "'save_video' is True, but there is no result_dir."
                raise RuntimeError(msg)
            if self._save_result:
                msg = "'save_results' is True, but there is no result_dir."
                raise RuntimeError(msg)
            if self._monitor_pilots:
                msg = "'monitor_pilots' is True, but there is no result_dir."
                raise RuntimeError(msg)
        else:
            self._result_dir.mkdir(exist_ok=True)
            if self._save_video:
                self._video_dir = self._result_dir / "videos"
                self._video_dir.mkdir(exist_ok=True)
            if self._monitor_pilots:
                self._monitor_plot_dir = self._result_dir / "monitor"
                self._monitor_plot_dir.mkdir(exist_ok=True)

    @property
    def scenario_name(self) -> str:
        return self._scenario.id

    def _log_info(self, msg: str) -> None:
        if self._i_job is not None:
            msg = f"Job {self._i_job:05d}: {msg}"

        logger.info(msg)

    def simulate(self) -> dict:
        self._log_info(f"Simulate scenario: {self.scenario_name}")

        # Set up simulation core that is responsible for vehicle dynamic simulation
        # TODO: use simple scenario file and specifiy the models there; input here is maybe simplescenario file or a list of needed agents?
        simulation_core = SimulationCore(
            self._scenario,
            live_plot=self._live_plot,
            write_video_path=self._video_dir,
            i_job=self._i_job,
        )

        # Init simulation and get first observation
        obs = simulation_core.reset()

        # Assign pilots to actors
        dt = obs["general"]["dt"]
        # TODO: use simple scenario file and specifiy the pilots there.
        pilots = {}
        for obj_id in obs["dynamic_objects"]:
            if obj_id == "ego":
                pilots[obj_id] = HighwayPilot(dt, ego_id=obj_id, silent=True)

            else:
                pilots[obj_id] = DummyPilot()

        # Run main simulation loop
        self._log_info("Run simulation")
        t0 = time()

        info_list = []
        elapsed_time_steps = 0
        done = False
        while not done:
            self._log_info(f"Step main loop at time_step {elapsed_time_steps}")

            # Collect actions from pilots
            self._log_info("Step pilots")
            cur_actions = {}
            for obj_id, pilot in pilots.items():
                cur_actions[obj_id] = pilot.step(obs)
                self._log_info(
                    f"Step pilot of '{obj_id}' ({pilot.__class__.__name__}) -> action: ({cur_actions[obj_id][0]:.3f}, {cur_actions[obj_id][1]:.3f})"
                )

            # Update simulation core
            self._log_info("Step SimulationCore")
            obs, _, done, info = simulation_core.step(cur_actions)

            info_list.append(info)
            elapsed_time_steps += 1

        # Reset again to make sure the video is saved properly
        simulation_core.reset(visualize=False)

        execution_time = time() - t0
        self._log_info("Simluation done")
        simulated_time = info["time_step"] * dt
        fps = simulated_time / execution_time
        self._log_info(
            f"Execution time: {execution_time:.2f}s, simulated time: {simulated_time:.2f}s, fps: {fps:.2f}"
        )

        # Reason
        termination_reason = next(
            name for name, status in info["termination_status"].items() if status
        )
        available_termination_reasons = list(info["termination_status"].keys())
        self._log_info(
            f"Termination reason: {termination_reason} (from: {available_termination_reasons})"
        )

        if self._monitor_pilots:
            for obj_id, pilot in pilots.items():
                if isinstance(pilot, HighwayPilot):
                    pilot.create_monitor_plots(
                        self._monitor_plot_dir, f"{obj_id}_{self.scenario_name}"
                    )

        # Aggregate dhw etc.
        dhws = [elem["dhw"] for elem in info_list]
        thws = [elem["thw"] for elem in info_list]
        ttcs = [elem["ttc"] for elem in info_list]

        dhw_min = np.min(dhws)
        thw_min = np.min(thws)
        ttc_min = np.min(ttcs)

        # Create result object
        ego_states = []
        for cur_observations, cur_actions in zip(
            simulation_core.observation_list[:-1], simulation_core.action_list
        ):
            cur_ego_observation = cur_observations["dynamic_objects"]["ego"]
            cur_ego_action = cur_actions["ego"]

            cur_ego_state = {}
            cur_ego_state.update(cur_ego_observation)
            cur_ego_state["action"] = cur_ego_action

            ego_states.append(cur_ego_state)

        res = {
            "scenario_name": self._scenario.id,
            "termination_reason": termination_reason,
            "termination_status": info["termination_status"],
            "termination_time_step": info["time_step"],
            "dhw_min": dhw_min,
            "thw_min": thw_min,
            "ttc_min": ttc_min,
            "ego_states": ego_states,
        }

        # Save to file
        if self._save_result:
            export_file = self._result_dir / f"{self.scenario_name}_results.json"
            with export_file.open("wb") as f:
                s = orjson.dumps(
                    res,
                    option=orjson.OPT_NAIVE_UTC
                    | orjson.OPT_SERIALIZE_NUMPY
                    | orjson.orjson.OPT_INDENT_2,
                )
                f.write(s)

        if self._i_job is not None:
            res = {str(self._i_job): res}

        return res
