# UUVSim Benchmark: Performance Evaluation Against LOTUSim

## Overview

This repository contains the benchmarking framework and analysis scripts used to evaluate the performance of the [**UUV Simulator (UUVSim)**](https://github.com/uuvsimulator/uuv_simulator) and compare it with **LOTUSim** (its BlueROVS).  
The objective is to assess simulation scalability, real-time performance, and computational efficiency when simulating multiple autonomous underwater vehicles (AUVs), specifically **BlueROVs**, under realistic environmental conditions.

Each simulation scenario involves an **underwater Gaussian current of 0.5 m/s**, while incrementally increasing the number of simulated agents.  
Two benchmarking modes are implemented:

- **Real-Time Performance Evaluation**
- **Accelerated-Time Performance Evaluation**

The structure is organised into two main folders, gz_sim_source used in [2. Setting up Gazebo](#2-setting-up-gazebo), and uuv_ws containing the UUVsim's workspace with our ROS package created to run the benchmark detailed in [Launch Benchmark](#4-launch-uuv-benchmark).

```text
├── gz_sim_source_files
│   ├── MainWindow.cc
│   ├── MainWindow.hh
│   ├── World.cc
│   └── WorldPrivate.hh
├── README.md
├── uuv_ws
│   ├── catkin_ws
│   │   └── src
│   │       ├── benchmark_bluerov2
│   │       │   ├── CMakeLists.txt
│   │       │   ├── package.xml
│   │       │   └── src
│   │       │       ├── benchmark_subscriber.py
│   │       │       ├── run_benchmark_uuv.py
│   │       │       └── spawn_multiple_rovs.py
│   │       ├── bluerov2
│   │       ├── gazebo_ros_pkgs
│   │       ├── uuv_simulator
.   .       └── ...
.   .
.   .
```

---

## Summary

- [1. Simulation Configuration](#1-simulation-configuration)
- [2. Setting up Gazebo](#2-setting-up-gazebo)
- [3. Set a Benchmark Mode](#3-set-a-benchmark-mode)
- [4. Launch UUV Benchmark](#4-launch-uuv-benchmark)

---

## 1. Simulation Configuration

All benchmarks were conducted using the [**UUV Simulator (UUVSim)**](https://github.com/uuvsimulator/uuv_simulator) running on **Ubuntu 18.04** with **Gazebo 9**, under the following hardware and software specifications:

| **Specification** | **UUVSim Configuration** |
|--------------------|--------------------------|
| **Operating System** | Ubuntu 18.04.6 LTS |
| **Processor** | Intel Core i7-11800H (up to 4.6 GHz) |
| **GPU** | NVIDIA GeForce RTX (8 GB VRAM) |
| **NVIDIA Driver** | 470.223.02 |
| **CUDA Version** | 11.4 |
| **Python Version** | 2.7.17 |
| **Gazebo Version** | 9 |
| **ROS Distribution** | ROS 1 Melodic |

Make sure to download and meet the requirements for Gazebo by clicking on the link. UUVSim will already be included in the repo through the submodules available in this repo.

---

## 2. Setting up Gazebo

Once the requirements and repositories are installed, Gazebo source code needs to be changed in order have calculate the metrics we need for this benchmark.

To do so, these 4 files from the `gz_sim_source_files` folder of the repo, need to be moved into Gazebo source code, and remplace the original files:

```text
├── gz_sim_source_files
│   ├── MainWindow.cc
│   ├── MainWindow.hh
│   ├── World.cc
│   └── WorldPrivate.hh
```

`MainWindow.cc` and `MainWindow.hh` will be moved and remplaced in **gazebo/gazebo/physics**.
`World.cc` and `WorldPrivate.hh` will be moved and remplaced in **gazebo/gazebo/gui**.

Make sure that the `.bashrc` includes these lines:

```shell
source /opt/ros/melodic/setup.bash
source /usr/local/share/gazebo/setup.sh

# Set environment variables so Gazebo finds the new sdformat 6.3+ when recompiled
#export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# To be able to use the recompiled gazebo
#export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PATH=/usr/local/bin:$PATH
```

Then open a new terminal at the root and type these commands:

```shell
cd gazebo
rm -rf build
mkdir build
cd build
cmake ..
make 
sudo make install
```

> **WARNING**:
With this set up, Gazebo will run and create three .csv per simulation, all saved in **/home/user_name/.ros** :

- `rtf.csv` : giving the value of the **real time factor** at each step on the simulation
- `update_times.csv` : giving the frequency of physics engine updates (`gz-server`)
- `fps.csv` : giving the value of the **frame per second** at each step on the simulation

## 3. Set a Benchmark Mode

For our benchmark to compare UUVsim to LOTUSim, we removed all sensors and actuators except the IMU.
You can decide which sensors and actuators to enable/disable in **~/your_path_to_repo/multi-agents-benchmark/benchmark_code/uuv_benchmark/uuv_ws/catkin_ws/src/bluerov2/bluerov2_description/urdf**

### Real Time

Modify the `empty_underwater.world` file located at  
**~/your_path_to_repo/multi-agents-benchmark/benchmark_code/uuv_benchmark/uuv_ws/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/worlds/empty_underwater.world**  
to adjust the simulation parameters **max step size**, **real-time update rate**, and **real-time factor** as follows:

```xml
<max_step_size>0.2</max_step_size>
<real_time_factor>1</real_time_factor>
<real_time_update_rate>5</real_time_update_rate>
```

### Accelerated Time

```xml
<max_step_size>0.03</max_step_size>
<real_time_factor>200</real_time_factor>
<real_time_update_rate>6666.667</real_time_update_rate>
```

## 4. Launch UUV Benchmark

Before launching an experiment to benchmark, please ensure you have:

- closed all other windows or apps
- disconnected the internet
- once gazebo is launched, extend the window in full screen to calculate the fps correctly and consistently

Open three terminals:

### Terminal 1

Start ROS:

```shell
roscore 
pkill -f roslaunch #at the end of the simulation, make sure to kill all the processes
```

### Terminal 2

Build the ROS packages and run the benchmark with `run_bnehcmark_uuv.py` from the ROS package `benchmark_bluerov2`:

```shell
cd ~/your_path_to_repo/multi-agents-benchmark/benchmark_code/uuv_benchmark/uuv_ws/catkin_ws
catkin_make
source devel/setup.bash
rosrun benchmark_bluerov2 run_benchmark_uuv.py
```

### Terminal 3

Start the underwater currents:

```shell
cd ~/your_path_to_repo/multi-agents-benchmark/benchmark_code/uuv_benchmark/uuv_ws/catkin_ws
source devel/setup.bash 
rosservice call /hydrodynamics/set_current_velocity "velocity: 0.5
horizontal_angle: 0.0
vertical_angle: 0.0"
```

> **WARNING***: after numerous simulation ran the .ros/log folder will start filling up. Don't hesitate to clean it:

```shell
rosclean check # → to check how much there is to clean
rosclean purge # → to clean the logs
```
