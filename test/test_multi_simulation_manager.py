import pytest
from pathlib import Path

from simple_simulation.simulation_manager import MultiSimulationManager

from test.check_test_result_format import check_test_result_format


class TestMultiSimulationManager:
    DATA_DIR = Path(__file__).parent / "test_data" / "challenger_a"

    RESULT_DIR = (
        Path(__file__).parent / "test_results" / "test_multi_simulation_manager"
    )
    RESULT_DIR.mkdir(exist_ok=True)

    def _evaluate(self, scenario_list, result_dir, n_workers=2):
        multi_simulation_manager = MultiSimulationManager(
            scenario_list, result_dir=result_dir, n_workers=n_workers
        )
        test_result = multi_simulation_manager.simulate()

        n_expected_results = 7
        n_actual_results = len(test_result)
        assert n_actual_results == n_expected_results, (
            f"Expected {n_expected_results} test results, got {n_actual_results}"
        )

        check_test_result_format(test_result, multiple_jobs=True)

    @pytest.mark.skip(
        reason="Testing multiprocessing seems to not work out of the box in gitlab runner."
    )
    def test_challenger_a(self):
        scenario_list = sorted(self.DATA_DIR.glob("*.json"))
        result_dir = self.RESULT_DIR / "challenger_a"
        self._evaluate(scenario_list, result_dir, n_workers=4)


if __name__ == "__main__":
    tester = TestMultiSimulationManager()
    tester.test_challenger_a()
