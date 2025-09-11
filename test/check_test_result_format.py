def check_test_result_format(result: dict, multiple_jobs: bool = False):
    required_keys = [
        "scenario_name",
        "termination_reason",
        "termination_status",
        "termination_time_step",
        "dhw_min",
        "thw_min",
        "ttc_min",
        "ego_states",
    ]

    if multiple_jobs:
        result_per_job = result
    else:
        result_per_job = {0: result}

    for job_result in result_per_job.values():
        for key in required_keys:
            assert key in job_result, f"Missing key '{key}'"

    return True
