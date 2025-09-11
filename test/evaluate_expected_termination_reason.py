from simulation_manager import SingleSimulationManager


def evaluate_expected_termination_reason(scenario, expected_termination_reason, result_dir):

    # Init simulation manager
    simulation_manager = SingleSimulationManager(scenario, result_dir)

    # Run simulation
    result = simulation_manager.simulate()

    # Read actual termination reason
    actual_termination_reason = result["termination_reason"]

    # Compare with expected
    assert actual_termination_reason == expected_termination_reason, f"Scenario '{simulation_manager.scenario_name}': Expected termination reason is {expected_termination_reason}, actual termination reason is {actual_termination_reason}"

    return result
