# MAS Benchmark

This repository contains benchmarking code and results for three simulators: LOTUSim, UUVSim, and LRAUVSim. The structure is organized into two main folders:

```text
├── benchmark_code
│   ├── lotusim_benchmark
│   ├── lrauvsim_benchmark
│   └── uuv_benchmark
└── benchmark_results
    ├── src
```

Clone the repository in the correct location:

```sh
git clone --recurse-submodules https://github.com/naval-group/LOTUSim-multi-agents-benchmark
```

Or after cloning:

```sh
git submodule update --init --recursive
```

## Benchmark_code

This folder contains the code to run benchmarks for all three simulators. Each simulator has its own subfolder:

- `lotusim_benchmark`
- `lrauvsim_benchmark`
- `uuv_benchmark`

> **Note**: Each subfolder includes a README explaining how to install the necessary dependencies and run the MAS benchmark for the simulator concerned.

## Benchmark_results

This project processes simulation benchmark data to generate comparative plots of performance metrics (RTF) across multiple simulators: LOTUSim, UUVSim, and LRAUVSim.

> **Note**: The details on how to process the results and visualize them are explained in the README located inside this folder.
