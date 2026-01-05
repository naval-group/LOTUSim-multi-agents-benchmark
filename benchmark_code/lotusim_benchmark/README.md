# LOTUSim Multi-Agent Benchmark

This repository contains resources to run and evaluate the multi-agent benchmark for LOTUSim.

**Repository structure:**

```text
├── docker_images
├── linux_executable/       # Prebuilt Linux executable (Unity)
│   └── Benchmark
└── README.md
```

---

## Docker Images

Get the Docker image containing the Docker build configurations and related resources used to create ready-to-use container images for the **Lotusim benchmark environment**.

```bash
docker pull juliettegrosset/lotusim_scenario:mas_benchmark
```

These images are designed to:

- Simplify the setup of **Lotusim** and **ROS 2 dependencies**.  
- Provide a **preconfigured runtime** for running the benchmark without requiring local installations.  
- Ensure **reproducible experiments** across different systems.  

---

### Docker Images Overview

---

### Launching the Docker Container

#### 1. Allow X Server Access

Before running the container, allow local GUI applications to connect to your X server:

```sh
xhost +
```

#### 2. Load the Docker Image

If you have a saved Docker image (.tar), load it into Docker:

```sh
docker load -i lotusim_benchmark.tar
```

#### 3. Run the Container

Launch the container with X11 display forwarding and network host access:

```sh
docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    lotusim_scenario:mas_benchmark
```

### Attaching to the Container

You can attach to the running container from VSCode or another IDE:

1. Open VSCode.

2. Install the Remote - Containers extension (now part of `Dev Containers` extension)

3. Use Attach to Running Container and select the **lotusim_scenario:mas_benchmark** container.

This allows you to edit files, run scripts, and debug inside the container interactively.

### Configuring benchmark.sh

Before running the benchmark, you may need to adjust the simulation parameters in `lotusim_generic_scenario/src/benchmark_mas/executable/benchmark.sh.`

All the different config files available are located in **lotusim-generic-scenario/simulation_run/config**

#### Configuration Section

```sh
# -------------------- Configuration --------------------
# Choose the configuration files to run

# List of configuration JSON files
#CONFIG_FILES=("lrauv_sensors.json")         # to run the benchmark with gazebo as rendering
CONFIG_FILES=("lrauv_sensors_unity.json")   # to run the benchmark with unity as rendering (check the following section paragraph)

# Uncomment the line below to run multiple configs
# CONFIG_FILES=("bluerov_gui.json" "bluerov_no-gui.json" "bluerov_sensors.json" ... )

# Agent model type. Options: "lrauv" or "bluerov"
MODEL_TYPE="lrauv"

# YAML configuration for dynamics. Options: "lrauv-ekman.yml" or "BlueROV2.yml"
YML_FILE="lrauv-ekman.yml"

# Number of agents for each simulation
AGENT_COUNTS=(5)

# Maximum simulation time in seconds
MAX_SIM_TIME=20
```

Notes:

- Adjust `CONFIG_FILES` to select which scenarios to run.
- Set `MODEL_TYPE` and `YML_FILE` according to the agent you want to benchmark.
- Change `AGENT_COUNTS` and `MAX_SIM_TIME` to control simulation scale and duration.

---

## Linux Executable

- Located in the `linux_executable/` folder.
- Contains the precompiled Unity executable for Linux: **Benchmark**

### Running Benchmark with Unity Visualization

If you want to run simulations with **Unity visualization**, the workflow is:

1. Launch the Unity executable (linux_executable/Benchmark) on the **host system**:

    - Put copy/paste the executable on the Desktop
    - Open a terminal from the Desktop and give the permissions to the executable (make sure it corespond to the name of the executable):

    ```shell
    chmod + Benchmark.x86_64
    ```

    - Run the Unity executable:

    ```shell
    ./Benchmark.x86_64
    ```

2. Then, run `benchmark.sh` **inside the Docker container**.
3. Once the benchmark is run, click on the Unity executable window to see the updates

> Unity must be running first because the benchmark script connects to it to display and control the simulation.
