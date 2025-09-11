from pathlib import Path

from test.evaluate_expected_termination_reason import evaluate_expected_termination_reason


class TestChallengerA:

    DATA_DIR = Path(__file__).parent / "test_data" / "challenger_a"

    RESULT_DIR = Path(__file__).parent / "test_results" / "challenger_a"
    RESULT_DIR.mkdir(exist_ok=True)

    EXPECTED_TERMINATION_REASONS = {
        "a0": "is_goal_reached",
        "a1": "is_standstill",
        "a2": "is_collision",
        "a3": "is_goal_reached",
        "a4": "is_goal_reached",
        "a5": "is_goal_reached",
        "standstill": "is_standstill"
    }

    def _evaluate(self, scenario_name: str):

        evaluate_expected_termination_reason(
            self.DATA_DIR / f"{scenario_name}.json",
            self.EXPECTED_TERMINATION_REASONS[scenario_name],
            self.RESULT_DIR
        )

    def test_challenger_a0(self):
        self._evaluate("a0")

    def test_challenger_a1(self):
        self._evaluate("a1")

    def test_challenger_a2(self):
        self._evaluate("a2")

    def test_challenger_a3(self):
        self._evaluate("a3")

    def test_challenger_a4(self):
        self._evaluate("a4")

    def test_challenger_a5(self):
        self._evaluate("a5")

    def test_challenger_standstill(self):
        """
        In this scenario, there is only one lane and the challenger comes to a standstill.
        The pilot should come to a standstill to avoid the crash.
        """
        self._evaluate("standstill")


if __name__ == "__main__":
    tester = TestChallengerA()
    tester.test_challenger_a0()
