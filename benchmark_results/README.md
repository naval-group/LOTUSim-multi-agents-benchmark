# Results Benchmarks

This project processes simulation benchmark data to generate comparative plots of performance metrics (RTF) across multiple simulators: LOTUSim, UUVSim, and LRAUVSim.

## Folder Structure

```text
├── pyproject.toml          # Project configuration
├── README.md               # This file
├── requirements.txt        # Python dependencies
├── src
│   └── benchmark_results   # Core scripts for processing benchmarks
│       ├── benchmark_accelerated_time  # General benchmark scripts
│       ├── benchmark_bluerov           # UUVSim-specific scripts
│       ├── benchmark_lrauv             # LRAUVSim-specific scripts
│       ├── __init__.py                 # Package initialization
│       └── py.typed                     # Type hint marker
└── uv.lock                  # Dependency lock file
```

**Folder details**:

- `benchmark_accelerated_time/`: Process general accelerated-time benchmarks from UUVSim, LRAUVSim and LOTUSim
- `benchmark_bluerov/`: Process UUVSim/LOTUSim (BlueROV) benchmark results.
- `benchmark_lrauv/`: Process LRAUVSim/LOTUSim (LRAUVSim) benchmark results.

**Inside each folder**:

- Raw input data and CSV summaries: `./data/`
- Generated plots: `./results/`
- Python scripts to process data and generate plots
