import json

from pathlib import Path
from simple_scenario import Scenario

from simple_simulation.simulation_manager import SingleSimulationManager

from test.check_test_result_format import check_test_result_format


class TestSingleSimulationManager:
    DATA_DIR = Path(__file__).parent / "test_data"

    RESULT_DIR = (
        Path(__file__).parent / "test_results" / "test_single_simulation_manager"
    )
    RESULT_DIR.mkdir(exist_ok=True)

    XML_SCENARIO_PATH = DATA_DIR / "ZAM_ValidationScenario+No+00001-0_0_T-0.xml"
    SIMPLE_SCENARIO_CONFIG_PATH = DATA_DIR / "test.json"

    def _evaluate(self, scenario, *args, **kwargs):
        simulation_manager = SingleSimulationManager(scenario, *args, **kwargs)
        result = simulation_manager.simulate()

        check_test_result_format(result, "i_job" in kwargs)

    def test_config_file(self):
        self._evaluate(self.SIMPLE_SCENARIO_CONFIG_PATH)

    def test_config(self):
        with self.SIMPLE_SCENARIO_CONFIG_PATH.open("r") as f:
            config = json.load(f)

        self._evaluate(config)

    def test_cr_file(self):
        self._evaluate(self.XML_SCENARIO_PATH)

    def test_scenario_object(self):
        scenario_object = Scenario.from_x(self.SIMPLE_SCENARIO_CONFIG_PATH)

        self._evaluate(scenario_object)

    def test_i_job(self):
        self._evaluate(self.SIMPLE_SCENARIO_CONFIG_PATH, i_job=100)

    def test_export(self):
        self._evaluate(
            self.SIMPLE_SCENARIO_CONFIG_PATH,
            self.RESULT_DIR / "test_export",
            save_result=True,
        )

    def test_video(self):
        self._evaluate(
            self.SIMPLE_SCENARIO_CONFIG_PATH,
            self.RESULT_DIR / "test_video",
            save_video=True,
        )

    def test_monitor_pilots(self):
        self._evaluate(
            self.SIMPLE_SCENARIO_CONFIG_PATH,
            self.RESULT_DIR / "test_monitor_pilots",
            monitor_pilots=True,
        )


if __name__ == "__main__":
    tester = TestSingleSimulationManager()
    tester.test_video()
