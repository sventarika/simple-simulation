import json
import re
import pytest

from pathlib import Path

from simple_scenario import Scenario
from simulation_manager import SingleSimulationEvaluator, MultiSimulationManager, CombinedTestReport
from test.evaluate_expected_termination_reason import evaluate_expected_termination_reason


class TestBaNeunzig:

    DATA_DIR = Path(__file__).parent / "test_data" / "ba_neunzig"
    LIMITS_DIR = Path(__file__).parent / "test_data" / "limits"

    RESULT_DIR = Path(__file__).parent / "test_results" / "ba_neunzig"
    RESULT_DIR.mkdir(exist_ok=True)

    EXPECTED_TERMINATION_REASONS = {
        "deceleration1" : "is_goal_reached",
        "deceleration2" : "is_goal_reached",
        "deceleration3" : "is_standstill",
        "deceleration4" : "is_standstill",
        "deceleration5" : "is_standstill",
        "deceleration6" : "is_standstill",
        "deceleration7" : "is_collision",
        "deceleration8" : "is_collision",
        "deceleration9" : "is_collision",
        "cutin1": "is_goal_reached",
        "cutin2": "is_goal_reached",
        "cutin3": "is_goal_reached"
    }

    def _evaluate(self, scenario_name: str, scenario_dir: str):

        scenario_config = self.DATA_DIR / scenario_dir / f"{scenario_name}.json"
        scenario = Scenario.from_x(scenario_config)

        # Read limits
        limits_file = self.LIMITS_DIR / f"{scenario_name}.json"
        with limits_file.open("r") as f:
            scenario_limits = json.load(f)

        complete_result_dir = self.RESULT_DIR / scenario_dir

        test_result = evaluate_expected_termination_reason(
            scenario_config,
            self.EXPECTED_TERMINATION_REASONS[scenario_name],
            complete_result_dir
        )

        # Evaluate test result
        # TODO: Take ChallengeDescription instead!
        scenario_description = {
            "expected_termination_reason": self.EXPECTED_TERMINATION_REASONS[scenario_name],
            "scenario_limits": scenario_limits
        }

        evaluation = SingleSimulationEvaluator(scenario, scenario_description, test_result, complete_result_dir)
        evaluation.create_test_report()

        return evaluation

    def test_deceleration1(self):
        self._evaluate("deceleration1", "deceleration")

    def test_deceleration2(self):
        self._evaluate("deceleration2", "deceleration")

    def test_deceleration3(self):
        self._evaluate("deceleration3", "deceleration")

    def test_deceleration4(self):
        self._evaluate("deceleration4", "deceleration")

    def test_deceleration5(self):
        self._evaluate("deceleration5", "deceleration")

    def test_deceleration6(self):
        self._evaluate("deceleration6", "deceleration")

    def test_deceleration7(self):
        self._evaluate("deceleration7", "deceleration")

    def test_deceleration8(self):
        self._evaluate("deceleration8", "deceleration")

    def test_deceleration9(self):
        self._evaluate("deceleration9", "deceleration")

    def test_cutin1(self):
        self._evaluate("cutin1", "cutin")

    def test_cutin2(self):
        self._evaluate("cutin2", "cutin")

    def test_cutin3(self):
        self._evaluate("cutin3", "cutin")

    @pytest.mark.skip(reason="Testing multiprocessing seems to not work out of the box in gitlab runner.")
    def test_combined_report(self):

        result_dir = self.RESULT_DIR / "test_combined_report"

        scenario_list = []
        scenario_dict = {}
        for scenario_name in self.EXPECTED_TERMINATION_REASONS:
            scenario_dir = re.match("([a-zA-z]+)[0-9]+$", scenario_name).groups()[0]
            scenario_file = self.DATA_DIR / scenario_dir / f"{scenario_name}.json"
            scenario_list.append(scenario_file)
            scenario_dict[scenario_name] = scenario_file

        multi_simulation_manager = MultiSimulationManager(scenario_list, result_dir=result_dir, n_workers=1)
        test_result = multi_simulation_manager.simulate()

        evaluator_list = []
        for job_result in test_result.values():

            scenario_name = job_result["scenario_name"]

            # Read limits
            limits_file = self.LIMITS_DIR / f"{scenario_name}.json"
            with limits_file.open("r") as f:
                scenario_limits = json.load(f)

            # Evaluate test result
            # TODO: Take ChallengeDescription instead!
            scenario_description = {
                "expected_termination_reason": self.EXPECTED_TERMINATION_REASONS[scenario_name],
                "scenario_limits": scenario_limits
            }

            evaluator = SingleSimulationEvaluator(scenario_dict[scenario_name], scenario_description, job_result, result_dir)
            evaluator.create_test_report()

            evaluator_list.append(evaluator)

        combined_test_report = CombinedTestReport(result_dir, evaluator_list)
        combined_test_report.create_combined_report()


if __name__ == "__main__":
    tester = TestBaNeunzig()
    tester.test_combined_report()
