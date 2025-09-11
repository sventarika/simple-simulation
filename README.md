# simple-scenario

## Initial setup

:bulb: You should use Linux, have docker installed, and have your ssh key set up.

1. Set up your GPG key and configure VSCode by going through the instructions in [vscode-docker-example](https://gitlab.ika.rwth-aachen.de/lva/vscode-docker-example).

2. Clone this repository.

```bash
git clone git@gitlab.ika.rwth-aachen.de:fb-fi/simulation/simple-simulation/simulation-manager.git
```

3. Change directory.

```bash
cd simulation-manager
```

4. Initialize submodules
```bash
git submodule init
```

5. Update submodules
```bash
git submodule update --depth 1
```

## Open the repository

1. Open the folder in VSCode (`Ctrl + K, Ctrl + O`)

2. Open the folder in a dev container (`Ctrl+Shift+P`: `Dev Containers: Rebuild and reopen in container`)

    You are probably asked for your gpg key password and it may take some time afterwards, because the docker image is downloaded from gitlab.

3. After you entered the dev container, you may have to accept the unsecure git subrepos (pop-up in the lower right corner, only relevant when being `root` in the devcontainer)

## Run some simulations

:bulb: You should first complete the initial setup.

1. Open the file `test/test_challenger_a.py` and run it by clicking on the run arrow in the top right corner.

2. There should be some videos in the folder `test/test_results/challenger_a/`.

You can watch them directly in VSCode.

## Use CoinHSL MA27 linear solver (optional)

Much faster than the standard one, but protected by personal license.

1. Request HSL Archive License [here](https://licences.stfc.ac.uk/product/coin-hsl-archive) and download the `coinhsl-archive-2022.12.02.tar.gz` tarball.

2. Place `coinhsl-archive-2022.12.02.tar.gz` into the `coinhsl/` folder.

3. Build local dockerfile using the following command

    ```bash
    docker build --rm -t gitlab.ika.rwth-aachen.de:5050/fb-fi/simulation/simple-simulation/simulation-manager:2.1.4-coinhsl .
    ```

4. Adapt used dockerfile in `.devcontainer/devcontainer.json` to the new docker image.

**WARNING**: Do not upload that docker image, because the HSL Archive License permits redistribution!

# Submodules

## mpc-controller

MPC controller used by `pilots` in `simple-simulation`.

## pilots

Pilots for the `simple-simulation`.

## simulation-core

The lightweight simulation core for the `simple-simulation`.
Based on the [CommonRoad vehicle models](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models).

## simulation-manager

Main module managing simple-simulation simulation runs.
