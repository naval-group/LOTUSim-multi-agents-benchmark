#
# Copyright (c) 2025 Naval Group
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
"""
@file statistics_unity_bluerov.py
@author Naval Group

@brief LOTUSim vs UUVSim Update Comparison

@note Processes simulation metrics from LOTUSim (BlueROV) and UUVSim,
computes mean update times, saves LOTUSim summary CSV, and plots both.

@details
Inputs:
    - UUVSim CSV: data/UUVSim_results.csv
    - LOTUSim folders: data/lotusim/<config_name>/<num_agents>/simulation_1_raw_metrics.csv

Outputs:
    - results/bluerov_uuvsim_lotusim_update_plot.eps
    - results/bluerov_lotusim_summary.csv

@version 0.1
@date 2025-10-22
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import skew, kurtosis

# ==========================
# Configuration
# ==========================
DATA_DIR = "data"
RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)

UUVSIM_CSV = os.path.join(DATA_DIR, "UUVSim_results.csv")
LOTUSIM_RAW_FILENAME = "simulation_1_raw_metrics.csv"

MIN_AGENTS = 0
MAX_AGENTS = 500
SLICE_INDEX = 400  # Skip warm-up samples

# Thresholds
PHYSICS_LIMIT = 30
USER_PERCEPTION_LIMIT = 200
PLOT_XLIMIT = 500
CUSTOM_YTICKS = [0, 30, 50, 100, 150, 200]

# ==========================
# Utility Functions
# ==========================


def safe_numeric(arr):
    """Convert a list to numeric values, discarding invalid entries."""
    numeric_arr = pd.to_numeric(arr, errors="coerce")
    return numeric_arr[~np.isnan(numeric_arr)]


def metric_evaluation(metrics, name):
    """Compute descriptive statistics for a numeric dataset."""
    return {
        "Metric": name,
        "Mean": round(np.mean(metrics), 4),
        "Median": round(np.median(metrics), 4),
        "Standard Deviation": round(np.std(metrics), 4),
        "Variance": round(np.var(metrics), 4),
        "Min": round(np.min(metrics), 4),
        "Max": round(np.max(metrics), 4),
        "Skewness": round(skew(metrics), 4),
        "Kurtosis": round(kurtosis(metrics), 4),
    }


def generate_execution_stats(prep, pre, update, post, exec_time, slice_index=SLICE_INDEX):
    """Slice warm-up samples and compute execution statistics."""
    if len(update) <= slice_index:
        raise ValueError(f"Not enough samples to slice from index {slice_index}")

    # Slice lists
    prep, pre, update, post = prep[slice_index:], pre[slice_index:], update[slice_index:], post[slice_index:]
    exec_time = exec_time[slice_index:]

    # Convert to numeric arrays
    prep, pre, update, post, exec_time = map(safe_numeric, [prep, pre, update, post, exec_time])

    # Compute statistics
    stats = [
        metric_evaluation(prep, "Preparation Update (ms)"),
        metric_evaluation(pre, "Pre-Update (ms)"),
        metric_evaluation(update, "Update (ms)"),
        metric_evaluation(post, "Post-Update (ms)"),
        metric_evaluation(exec_time, "Execution Time (ms)"),
    ]
    return pd.DataFrame(stats).round(2)


# ==========================
# Data Loading Functions
# ==========================


def load_lotusim_folder(folder_path):
    """Load a single LOTUSim configuration folder and compute mean Update per agent."""
    data = []
    for folder in sorted(os.listdir(folder_path), key=lambda x: int(x) if x.isdigit() else float("inf")):
        if not folder.isdigit():
            continue
        agents = int(folder)
        if agents < MIN_AGENTS or agents > MAX_AGENTS:
            continue

        raw_path = os.path.join(folder_path, folder, LOTUSIM_RAW_FILENAME)
        if not os.path.isfile(raw_path):
            continue

        try:
            df = pd.read_csv(raw_path)
            df.columns = df.columns.str.strip()
            prep = df["Preparation Update (ms)"].tolist()
            pre = df["Pre-Update (ms)"].tolist()
            update = df["Update (ms)"].tolist()
            post = df["Post-Update (ms)"].tolist()
            exec_time = df["Execution Time (ms)"].tolist()

            exec_table = generate_execution_stats(prep, pre, update, post, exec_time)
            update_row = exec_table[exec_table["Metric"] == "Update (ms)"]
            if not update_row.empty:
                mean_update = update_row["Mean"].values[0]
                if not (mean_update < 20 and agents > 300):  # filter unrealistic data
                    data.append({"Agents": agents, "LOTUSim": mean_update})
        except Exception as e:
            print(f"⚠️ Failed at {raw_path}: {e}")

    return pd.DataFrame(data).sort_values("Agents").reset_index(drop=True)


def load_all_lotusim_data(base_dir):
    """Load all LOTUSim configuration folders and aggregate mean updates by agent."""
    lotusim_folders = [
        os.path.join(base_dir, f) for f in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, f))
    ]
    dfs = [load_lotusim_folder(folder) for folder in lotusim_folders if not load_lotusim_folder(folder).empty]
    if dfs:
        df_combined = pd.concat(dfs).groupby("Agents").mean().reset_index()
        return df_combined
    return pd.DataFrame(columns=["Agents", "LOTUSim"])


def save_lotusim_summary_csv(df_lotus, filename="bluerov_lotusim_summary.csv"):
    """Save LOTUSim summary CSV."""
    path = os.path.join(RESULTS_DIR, filename)
    df_lotus.to_csv(path, index=False)
    print(f"✅ LOTUSim summary saved to {path}")


def load_uuvsim_csv(csv_path):
    """Load UUVSim CSV file."""
    return pd.read_csv(csv_path)


# ==========================
# Plotting Function
# ==========================


def plot_update_comparison(df_uuv, df_lotus):
    """Plot UUVSim vs LOTUSim mean updates with thresholds."""
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot curves
    ax.plot(
        df_uuv["Number of Agents"], df_uuv["Mean Update (ms)"], marker="o", linestyle="-", color="blue", label="UUVSim"
    )
    ax.plot(df_lotus["Agents"], df_lotus["LOTUSim"], marker="o", linestyle="-", color="teal", label="LOTUSim")

    # Horizontal thresholds
    ax.axhline(PHYSICS_LIMIT, color="purple", linestyle=":", linewidth=1)
    ax.text(-90, PHYSICS_LIMIT, "Physics\nLimit", color="purple", fontsize=16, va="center", ha="left")

    ax.axhline(USER_PERCEPTION_LIMIT, color="red", linestyle=":", linewidth=1)
    ax.text(-90, USER_PERCEPTION_LIMIT, "User\nPerception\nLimit", color="red", fontsize=16, va="center", ha="left")

    # Vertical limit lines
    vertical_lines = [
        (30, 30, "30", "teal"),
        (130, 30, "130", "blue"),
        (333, 94, "333\nRAM limits", "blue"),
        (450, 202, "450", "teal"),
    ]
    line_colors = ["purple", "purple", "red", "red"]
    for idx, (x, ymax, label, label_color) in enumerate(vertical_lines):
        ax.plot([x, x], [0, ymax], color=line_colors[idx], linestyle=":", linewidth=1)
        ax.text(x, -10, label, ha="center", va="top", color=label_color, fontsize=16)

    # Dynamic Y-axis
    max_val = max(df_uuv["Mean Update (ms)"].max(), df_lotus["LOTUSim"].max(), USER_PERCEPTION_LIMIT)
    ax.set_ylim(0, max_val * 1.1)
    ax.set_yticks([tick for tick in CUSTOM_YTICKS if tick <= max_val * 1.1])

    # Labels and grid
    ax.set_xlabel("nb_agents", fontsize=18)
    ax.set_ylabel("Update (ms)", fontsize=18)
    ax.set_xlim(0, PLOT_XLIMIT)
    ax.grid(True, linestyle=":", linewidth=0.7)
    ax.tick_params(axis="both", labelsize=16)
    ax.legend(loc="upper left", bbox_to_anchor=(0.2, 0.9), fontsize=16, title_fontsize=14)

    # Save figure
    plot_path = os.path.join(RESULTS_DIR, "bluerov_uuvsim_lotusim_update_plot.eps")
    plt.tight_layout()
    plt.savefig(plot_path, format="eps")
    plt.show()
    print(f"✅ Plot saved to {plot_path}")


# ==========================
# Main Execution
# ==========================


def main():
    # Load UUVSim CSV
    df_uuv = load_uuvsim_csv(UUVSIM_CSV)

    # Load LOTUSim data
    LOTUSIM_BASE_DIR = os.path.join(DATA_DIR, "lotusim")
    df_lotus = load_all_lotusim_data(LOTUSIM_BASE_DIR)

    # Save LOTUSim summary CSV
    save_lotusim_summary_csv(df_lotus)

    # Plot comparison
    plot_update_comparison(df_uuv, df_lotus)


if __name__ == "__main__":
    main()
