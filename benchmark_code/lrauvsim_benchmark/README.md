# LRAUVSim Multi-Agent Benchmark

## Overview

This repository contains the benchmarking framework and analysis scripts used to evaluate the performance of the [**LRAUV Simulator (LRAUVSim)**](https://github.com/osrf/lrauv) and compare it with **LOTUSim** (its BlueROVS).  
The objective is to assess simulation scalability, real-time performance, and computational efficiency when simulating multiple autonomous underwater vehicles (AUVs), specifically **LRAUVs**, under realistic environmental conditions.

Each simulation scenario involves an **underwater Gaussian current of 0.5 m/s**, while incrementally increasing the number of simulated agents.  
Two benchmarking modes are implemented:

- **Real-Time Performance Evaluation**
- **Accelerated-Time Performance Evaluation**

The structure is organised into three main folders, gz_sim_source used in [2. Setting up Gazebo](#2-setting-up-gazebo), lrauv_gazebo_plugins_source_files used in [3. Setting up Gazebo Plugins](#3-setting-up-gazebo-plugins) and lrauv_ws containing the LRAUV's workspace with detailed in [Launch Benchmark](#5-launch-lrauvsim-benchmark).

```text
.
├── gazebo_source_files
│   ├── MainWindow.cc
│   ├── WorldStats.cc
│   └── WorldStats.hh
├── lrauv_gazebo_plugins_source_files
│   ├── empty_environment.sdf
│   ├── HydrodynamicsPlugin.cc
│   ├── model.sdf
│   ├── SpawnPanelPlugin.cc
│   └── SpawnPanelPlugin.qml
├── lrauv_ws
│   └── src
│       ├── gz
.       └── ...
.   
.   
```

## Summary

- [1. Simulation Configuration](#1-simulation-configuration)
- [2. Setting up Gazebo](#2-setting-up-gazebo)
- [3. Setting up Gazebo Plugins](#3-setting-up-gazebo-plugins)
- [4. Set a Benchmark Mode](#4-set-a-benchmark-mode)
- [5. Launch LRAUVSim Benchmark](#5-launch-lrauvsim-benchmark)

---

## 1. Simulation Configuration

All benchmarks were conducted using the [**LRAUV Simulator**](https://github.com/osrf/lrauv/wiki/Installation) running on **Ubuntu 20.04** with [**Gazebo Garden**](https://gazebosim.org/docs/garden/install_ubuntu_src/), under the following hardware and software specifications:

| **Specification** | **LRAUVSim Configuration** |
|--------------------|--------------------------|
| **Operating System** | Ubuntu 20.04.66 LTS |
| **Processor** | Intel Core i7-12650H (up to 2.7 GHz) |
| **GPU** | NVIDIA GeForce RTX (8 GB VRAM) |
| **NVIDIA Driver** | 570.133.07 |
| **CUDA Version** | 12.8 |
| **Python Version** | 3.8.10 |
| **Gazebo Version** | Garden |

Make sure to download and meet the requirements for Gazebo by clicking on the links. LRAUVSim will already be included in the repo through the submodules available in this repo.

## 2. Setting up Gazebo

Once the requirements and repositories are installed, Gazebo source code needs to be changed in order have calculate the metrics we need for this benchmark.

To do so, these 3 files from the `gz_sim_source_files` folder of the repo, need to be moved into Gazebo source code, and remplace the original files:

```text
MainWindow.cc
WorldStats.cc
WorldStats.hh
```

Here **/workspace** correspond to Gazebo workspace.

`MainWindow.cc` will be moved and remplaced in **workspace/src/gz-gui/src**.
`WorldStats.cc` and `WorldStats.hh` will be moved and remplaced in **workspace/src/gz-gui/src/plugins/world_stats**.

For Gazebo Garden, the workspace needs to be sourced every time a new terminal is used, or can be put in the `.bashrc`:

```shell
. ~/workspace/install/setup.bash
```

Then open a terminal in gazebo's folder, and recompile Gazebo with the new files:

```shell
rm -rf build install log
colcon build --merge-install
colcon build --packages-select gz-gui7 --merge-install
```

## 3. Setting up Gazebo Plugins

To perform the benchmark, some gazebo pluggins have been directly changed too, in order to integrate in the UI a buttom to spawn LRAUVs into the world.

Therefore, these 5 files from the `lrauv_gazebo_plugins_source_files` folder, need to be moved into LRAUV Gazebo plugins source code, and remplace the original files:

```text
empty_environment.sdf
HydrodynamicsPlugin.cc
model.sdf
SpawnPanelPlugin.cc
SpawnPanelPlugin.qml
```

`SpawnPanelPlugin.cc` and `SpawnPanelPlugin.qml` will be moved and remplaced in **/your_path_to_repo/multi-agents-benchmark/benchmark_code/lrauvsim_benchmark/lrauv_ws/src/lrauv/lrauv_gazebo_plugins/src**

`empty_environment.sdf` will be moved and remplaced in **/your_path_to_repo/multi-agents-benchmark/benchmark_code/lrauvsim_benchmark/lrauv_ws/src/lrauv/lrauv_gazebo_plugins/worlds**

`HydrodynamicsPlugin.cc` will be moved and remplaced in **/your_path_to_repo/multi-agents-benchmark/benchmark_code/lrauvsim_benchmark/lrauv_ws/src/lrauv/lrauv_gazebo_plugins/src**
→ pluggin to set the underwater current, by default, it has been set to 0.5m/s. It can be changed with this line:

```C++
 /// \brief Water current [m/s].
 public: gz::math::Vector3d waterCurrent {0.5, 0.0, 0.0};
 ```

`model.sdf` will be moved and remplaced in **/your_path_to_repo/multi-agents-benchmark/benchmark_code/lrauvsim_benchmark/lrauv_ws/src/lrauv/lrauv_description/models/tethys_equipped**
→ in this file, you can specify which sensors/actuators needs to be used.

## 4. Set a Benchmark Mode

For our benchmark to compare UUVsim to LOTUSim, we removed all sensors and actuators except the IMU.
You can decide which sensors and actuators to enable/disable in **~/your_path_to_repo/multi-agents-benchmark/benchmark_code/uuv_benchmark/uuv_ws/catkin_ws/src/bluerov2/bluerov2_description/urdf**

### Real Time

Modify the `empty_environment.sdf` file located at  
**~/your_path_to_repo/multi-agents-benchmark/benchmark_code/lrauvsim_benchmark/lrauv_gazebo_plugins_source_files**  
to adjust the simulation parameters **max step size**, **real-time update rate**, and **real-time factor** as follows:

```xml
<!-- For Real Time -->
<physics name="default" type="dart">
    <max_step_size>0.03</max_step_size>       
    <real_time_update_rate>33.33</real_time_update_rate>
    <real_time_factor>1.0</real_time_factor>   
</physics> 
```

### Accelerated Time

```xml
<!-- For Accelerated Time -->
<physics name="1ms" type="dart">
    <max_step_size>0.03</max_step_size>
    <real_time_factor>0</real_time_factor>
</physics>
```

> **Note**: rtf=0 translates as 'as fast as possible'

## 5. Launch LRAUVSim Benchmark

Before launching an experiment to benchmark, please ensure you have:

- closed all other windows or apps
- disconnected the internet
- once gazebo is launched, extend the window in full screen to calculate the fps correctly and consistently

Open one terminals:

### Terminal

Source the LRAUV workspace and launch the world to start the benchmark:

```shell
cd /your_path_to_repo/multi-agents-benchmark/benchmark_code/lrauvsim_benchmark/lrauv_ws/
colcon build 
# or if only the lrauv gazebo pluggin have been changed : 
#colcon build --packages-select lrauv_gazebo_plugins
source install/setup.bash
gz sim empty_environment.sdf
```

### LRAUV UI

Once **LRAUVSim** is launched, open the **Spawn LRAUVs** panel on the right-hand side. Expand it by dragging or clicking to reveal all available buttons, including **`Spawn!`** and **`Marie!`**.  

Next, enter the desired number of LRAUVs you wish to spawn for the benchmark, then click the **`Marie!`** button to create them in the simulation.  

Finally, press the **`Play`** button located at the bottom-left corner of the window to start the simulation.

![LRAUV Simulation Interface](benchmark_code/lrauvsim_benchmark/lrauv_ui.png)

> **WARNING**: With this set up, Gazebo will run and create three .csv per simulation, all saved in **/home/user_name/** :

- `rtf.csv` : giving the value of the **real time factor** at each step on the simulation
- `update_times.csv` : giving the frequency of physics engine updates (`gz-server`)
- `fps.csv` : giving the value of the **frame per second** at each step on the simulation
