from loguru import logger
from pathlib import Path
from simple_scenario import Scenario
from tqdm import tqdm

from simple_simulation.simulation_manager import SingleSimulationManager


def main() -> None:
    scenario_dir = Path(__file__).parent

    logger.info("Generate simulation run videos for example scenarios")

    for i in tqdm(range(3), desc="Execute simulation runs"):
        scenario_file = scenario_dir / f"example_scenario{i}.json"
        scenario = Scenario.from_x(scenario_file)
        scenario.render_gif(scenario_dir)

        sim_manager = SingleSimulationManager(
            scenario, result_dir=scenario_dir, save_video=True
        )
        sim_manager.simulate()

    logger.info("Simulation run videos saved to 'examples/videos/' directory")


if __name__ == "__main__":
    main()
