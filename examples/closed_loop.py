from pathlib import Path

from simple_simulation.simulation_manager import SingleSimulationManager


def run_closed_loop_simulation() -> None:
    data_dir = Path(__file__).parent.parent / "test" / "test_data" / "simulation_config"

    result_dir = Path(__file__).parent.parent / "results" / "closedloop"
    result_dir.mkdir(exist_ok=True)

    simulation_manager = SingleSimulationManager(
        simulation_config=data_dir / "simulation_config.json"
    )
    simulation_manager.simulate()


if __name__ == "__main__":
    run_closed_loop_simulation()
