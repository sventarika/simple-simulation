from pathlib import Path
from simple_simulation.simulation_manager import SingleSimulationManager


class CloseLoopSimulation:
    DATA_DIR = Path(__file__).parent / "test" / "test_data" / "simulation_config"

    RESULT_DIR = Path(__file__).parent / "results" / "closedloop"
    RESULT_DIR.mkdir(exist_ok=True)

    SIMPLE_SCENARIO_CONFIG_PATH = DATA_DIR / "highway.json"

    def run_simulation(self) -> None:
        simulation_manager = SingleSimulationManager(
            simulation_config=self.DATA_DIR / "simulation_config.json"
        )
        simulation_manager.simulate()


if __name__ == "__main__":
    simulation = CloseLoopSimulation()
    simulation.run_simulation()
