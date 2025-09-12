# simple-simulation

Execute simulation runs defined by [simple-scenario](https://github.com/ika-rwth-aachen/simple-scenario) for developing test scenario selection algorithms and testing prototype automated driving systems.

## Install

To use or develop `simple-simulation`, you must first clone the repository.

```bash
$ git clone git@gitlab.ika.rwth-aachen.de:fb-fi/simulation/simple-simulation/simple-simulation.git
$ cd simple-simulation
```

It is recommended to use [uv](https://docs.astral.sh/uv/getting-started/installation/) for package management and usage (see [With uv](#with-uv)).
If needed, this repository also comes with a `Dockerfile` for execution in docker containers (see [With docker](#with-docker)).

### With uv

Install requirements with

```bash
$ uv sync
```

To run a script, use

```bash
$ uv run /path/to/script.py
```

or directly use the python interpreter from the `.venv` folder in e.g. VSCode.

To run the tests, install the dev requirements with

```bash
$ uv sync --dev
```

and run the tests

```bash
$ uv run pytest
```

### With docker

1. Build docker image

```bash
$ docker build --rm -t gitlab.ika.rwth-aachen.de:5050/fb-fi/simulation/simple-simulation/simple-simulation:0.1.0 .
```

:bulb: The docker image is based on an image from [opencv-docker](https://gitlab.ika.rwth-aachen.de/fb-fi/misc/opencv-docker), because the opencv version from PyPi does not included the codec needed to watch videos generated with opencv directly in VSCode or any web browser (i.e., also not in GitLab issues).

2. Make sure that the docker image version specified in the `.devcontainer/devcontainer.json` matches the built docker image.

3. Open the folder in VSCode (`Ctrl + K, Ctrl + O`)

4. Open the folder in a dev container (`Ctrl+Shift+P`: `Dev Containers: Rebuild and reopen in container`)

(Optional) Use CoinHSL MA27 linear solver

Much faster than the standard one, but protected by personal license.

1. Request HSL Archive License [here](https://licences.stfc.ac.uk/product/coin-hsl-archive) and download the `coinhsl-archive-2022.12.02.tar.gz` tarball.

2. Place `coinhsl-archive-2022.12.02.tar.gz` into the `coinhsl/` folder.

3. Build local dockerfile using the following command

```bash
$ docker build --rm -t gitlab.ika.rwth-aachen.de:5050/fb-fi/simulation/simple-simulation/simulation-manager:0.1.0-coinhsl .
```

4. Adapt used dockerfile in `.devcontainer/devcontainer.json` to the new docker image.

**WARNING**: Do not upload that docker image to gitlab, because the HSL Archive License permits redistribution!

# Use

:bulb: You should first complete the [Installation](#install).

Run some simulations

1. Open the file `test/test_challenger_a.py` and run it by clicking on the run arrow in the top right corner.

2. There should be some videos in the folder `test/test_results/challenger_a/`.

You can watch them directly in VSCode.

# Dev

The following lists the main idea of the modules:

:bulb: For more details of the interaction between the different modules, please read [#1](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/simple-simulation/simple-simulation/-/issues/1).

* `simulation_manager/`: Main module handling all other modules to run the main simulation loop. (Previous and deprecated repo: [simulation-manager](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/simple-simulation/simulation-manager))
* `pilots/`: Pilots for the simulation actors. A pilot takes high-level decisions for lateral and longitudinal control for exactly one simulation actor based on the current situation in the simulation. Each pilot must be a subclass of `Pilot` in `pilots/pilot.py`. (The previous and deprecated repo [pilots](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/simple-simulation/pilots/-/tree/hidrive-models?ref_type=heads) contains the pilots used for the Hi-Drive safety impact assessment simulations).
* `mpc_controller/`: An mpc controller that is used by the `HighwayPilot` (and the Hi-Drive pilots) for making sure that the controlled vehicle will follow the reference trajectory. (Previous and deprecated repo [mpc-controller](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/simple-simulation/mpc-controller) contains more mpc controller versions and some scripts testing different CommonRoad models and MPC configurations. The currently used `MpcController` is named `AdvancedMpcController` there!).
* `simulation_core/`: The lightweight simulation core using [CommonRoad vehicle models](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models). In a perfect world, this can be swapped with an interface to esmini, VTD, Carla and all other modules can be still be used.
