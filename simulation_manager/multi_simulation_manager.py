from __future__ import annotations

import multiprocessing as mp
import orjson

from loguru import logger
from pathlib import Path
from time import process_time
from tqdm import tqdm

from simple_scenario import Scenario
from simulation_manager import SingleSimulationManager


class MultiSimulationManager:

    def __init__(self,
                 scenario_list: str | Path | dict | Scenario,
                 n_workers: int=1,
                 result_dir: str | None = None,
                 save_videos: bool=False,
                 save_results: bool=False,
                 monitor_pilots: bool=False
                 ) -> None:

        self._scenario_list = []
        # If input is Scenario object, need to take its config. Otherwise MP will not be possible!
        for scenario in scenario_list:
            if isinstance(scenario, Scenario):
                self._scenario_list.append(scenario.config)
            else:
                self._scenario_list.append(scenario)

        if int(n_workers) != n_workers or n_workers < 1:
            msg = "False value for n_workers"
            raise ValueError(msg)

        self._n_workers = n_workers

        self._save_results = save_results
        self._save_videos = save_videos
        self._monitor_pilots = monitor_pilots

        logger.info("MultiSimulationManager for {} scenarios", self._scenario_list)

        if len(self._scenario_list) == 0:
            msg = "No scenarios found for simulation"
            raise Exception(msg)

        self._monitor_plot_dir = None
        self._video_dir = None

        if result_dir is None:
            if self._save_videos:
                msg = "'save_video' is True, but there is no result_dir."
                raise RuntimeError(msg)
            if self._save_results:
                msg = "'save_results' is True, but there is no result_dir."
                raise RuntimeError(msg)
            if self._monitor_pilots:
               msg = "'monitor_pilots' is True, but there is no result_dir."
               raise RuntimeError(msg)
        else:
            self._result_dir = Path(result_dir)
            self._result_dir.mkdir(exist_ok=True)

            if self._monitor_pilots:
                self._monitor_plot_dir = self._result_dir / "monitor_plots"
                self._monitor_plot_dir.mkdir()

            if self._save_videos:
                self._video_dir = self._result_dir / "videos"
                self._video_dir.mkdir()

    def simulate(self) -> dict:

        logger.info("Simulate {} scenario(s)", len(self._scenario_list))

        t0 = process_time()

        # Collect arguments for all scenarios
        logger.info("Collect args for all runs")
        all_argument_lists = []
        for i_job, scenario in enumerate(self._scenario_list):
            all_argument_lists.append([scenario, self._result_dir, i_job, self._save_videos, self._save_results, self._monitor_pilots])

        all_test_results = {}

        if self._n_workers == 1:
            logger.info("Run sequentially")

            for arg_list in tqdm(all_argument_lists):
                test_result = self._simulate_single_scenario(*arg_list)
                all_test_results.update(test_result)
        else:

            n_max_cpu = mp.cpu_count() - 1
            n_workers = min(self._n_workers, n_max_cpu)

            logger.info("Run in parallel with {} workers (max_workers: {})", n_workers, n_max_cpu)

            with mp.Pool(n_workers, maxtasksperchild=1) as p:
                test_result_iterator = p.imap(MultiSimulationManager._unpack_argument_list_and_simulate_single_scenario, all_argument_lists, chunksize=1)

                for test_result in tqdm(test_result_iterator, total=len(all_argument_lists)):
                    all_test_results.update(test_result)

        logger.info("Simulation done")
        execution_time = process_time() - t0
        logger.info("Execution time for {} scenarios: {:.2f}s", len(all_argument_lists), execution_time)

        if self._save_results:

            all_test_results_file = self._result_dir / "all_test_results.json"
            with all_test_results_file.open("wb") as f:
                s = orjson.dumps(all_test_results, option=orjson.OPT_NAIVE_UTC | orjson.OPT_SERIALIZE_NUMPY | orjson.orjson.OPT_INDENT_2)
                f.write(s)

        return all_test_results

    @staticmethod
    def _unpack_argument_list_and_simulate_single_scenario(argument_list: list) -> dict:
        return MultiSimulationManager._simulate_single_scenario(*argument_list)

    @staticmethod
    def _simulate_single_scenario(scenario: str | Path | dict | Scenario, result_dir: Path, i_job: int, save_video: bool, save_result: bool, monitor_pilots: bool) -> dict:

        simulation_manager = SingleSimulationManager(scenario, result_dir=result_dir, i_job=i_job, live_plot=False, save_video=save_video, save_result=save_result, monitor_pilots=monitor_pilots)

        test_result = simulation_manager.simulate()

        return test_result


if __name__ == "__main__":

    challenger_scenarios = "data/challenger_scenario_examples"

    cfg = {
        "scenario_xml_dir": "test/test_data/challenger_a/experiment1",
        "first_scenario_idx": 0,
        "last_scenario_idx": -1,
        "create_monitor_plots": False,
        "create_video": False,
        "n_workers": 9,
        "show_live_plot": False,
    }

    MultiSimulationManager.run(parsed_args_dict=cfg)
