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
@file statistics_unity_lrauv.py
@author Naval Group

@brief LOTUSim vs LRAUVSim Update Comparison

@note Processes simulation metrics from LOTUSim (LRAUV) and LRAUVSim,
computes mean update times, and plots both on the same graph.

@details
Inputs:
    - LRAUVSim CSV: data/LRAUVSim_results.csv
    - LOTUSim folders: data/lotusim/<config_name>/<num_agents>/simulation_1_raw_metrics.csv

Outputs:
    - results/lrauv_uuvsim_lotusim_update_plot.eps
    - results/lrauv_lotusim_summary.csv

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
LOTUSIM_DIR = os.path.join(DATA_DIR, "lotusim", "lrauv_sensors_unity")
os.makedirs(RESULTS_DIR, exist_ok=True)

LRAUVSIM_CSV = os.path.join(DATA_DIR, "LRAUVSim_results.csv")
RAW_FILENAME = "simulation_1_raw_metrics.csv"
MIN_AGENTS = 0
MAX_AGENTS = 800
SLICE_INDEX = 400  # skip warm-up samples

# Thresholds
PHYSICS_LIMIT = 30
USER_PERCEPTION_LIMIT = 200

# ==========================
# Utility Functions
# ==========================


def safe_numeric(arr):
    """Convert a list to numeric values, discarding NaNs."""
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


def generate_statistical_table(prep, pre, update, post, exec_time, rtf, cpu, gpu):
    """
    Slice warm-up samples, clean numeric data, and compute statistics.
    Returns execution statistics table.
    """
    if len(update) <= SLICE_INDEX:
        raise ValueError(f"Not enough samples to slice from index {SLICE_INDEX}")

    prep, pre, update, post = map(lambda x: safe_numeric(x[SLICE_INDEX:]), [prep, pre, update, post])
    exec_time, rtf, cpu, gpu = map(lambda x: safe_numeric(x[SLICE_INDEX:]), [exec_time, rtf, cpu, gpu])

    execution_stats = [
        metric_evaluation(prep, "Preparation Update (ms)"),
        metric_evaluation(pre, "Pre-Update (ms)"),
        metric_evaluation(update, "Update (ms)"),
        metric_evaluation(post, "Post-Update (ms)"),
        metric_evaluation(exec_time, "Execution Time (ms)"),
    ]

    return pd.DataFrame(execution_stats).round(2)


# ==========================
# Data Loading Functions
# ==========================


def load_lotusim_data(folder_path):
    """
    Process a LOTUSim folder and compute mean Update (ms) for each agent count.
    Returns a DataFrame with columns ['Agents', 'LOTUSim'].
    """
    data = []
    for folder in sorted(os.listdir(folder_path), key=lambda x: int(x) if x.isdigit() else float("inf")):
        if not folder.isdigit():
            continue
        agents = int(folder)
        if agents < MIN_AGENTS or agents > MAX_AGENTS:
            continue

        raw_path = os.path.join(folder_path, folder, RAW_FILENAME)
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
            rtf = df["Real Time Factor (%)"].tolist()
            cpu = df["CPU Usage (%)"].tolist()
            gpu = df["GPU Usage (%)"].tolist()

            exec_table = generate_statistical_table(prep, pre, update, post, exec_time, rtf, cpu, gpu)
            update_row = exec_table[exec_table["Metric"] == "Update (ms)"]

            if not update_row.empty:
                mean_update = update_row["Mean"].values[0]
                # Filter unrealistic data for high agent counts
                if not (mean_update < 20 and agents > 300):
                    data.append({"Agents": agents, "LOTUSim": mean_update})
        except Exception as e:
            print(f"⚠️ Failed to process {raw_path}: {e}")

    return pd.DataFrame(data).sort_values("Agents").reset_index(drop=True)


def load_lrauvsim_csv(csv_path):
    """Load LRAUVSim CSV file into a DataFrame."""
    return pd.read_csv(csv_path)


# ==========================
# Function to save LOTUSim summary CSV
# ==========================


def save_lotusim_summary_csv(df_lotus, filename="lrauv_lotusim_summary.csv"):
    """
    Saves LOTUSim mean update times per agent count to a CSV.
    """
    results_path = os.path.join(RESULTS_DIR, filename)
    df_lotus.to_csv(results_path, index=False)
    print(f"✅ LOTUSim summary saved to {results_path}")


# ==========================
# Plotting Function
# ==========================


def plot_update_comparison(df_lrauv, df_lotus):
    """
    Plot LRAUVSim vs LOTUSim mean update times.
    Adds physics and user perception threshold lines.
    """
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot LRAUVSim
    ax.plot(
        df_lrauv["Number of Agents"],
        df_lrauv["Mean Update (ms)"],
        marker="o",
        linestyle="-",
        color="orange",
        label="LRAUVSim",
    )

    # Plot LOTUSim
    ax.plot(df_lotus["Agents"], df_lotus["LOTUSim"], marker="o", linestyle="-", color="teal", label="LOTUSim")

    # Threshold lines
    ax.axhline(PHYSICS_LIMIT, color="purple", linestyle=":", linewidth=1)
    ax.text(-130, PHYSICS_LIMIT, "Physics\nLimit", color="purple", fontsize=16, va="center", ha="left")

    ax.axhline(USER_PERCEPTION_LIMIT, color="red", linestyle="dotted", linewidth=1)
    ax.text(-135, USER_PERCEPTION_LIMIT, "User\nPerception\nLimit", color="red", fontsize=16, va="center", ha="left")

    # Vertical lines at key agent counts, stopping at thresholds
    ax.plot([55, 55], [0, PHYSICS_LIMIT], color="purple", linestyle=":", linewidth=1)
    ax.plot([65, 65], [0, USER_PERCEPTION_LIMIT], color="red", linestyle=":", linewidth=1)
    ax.plot([750, 750], [0, USER_PERCEPTION_LIMIT], color="red", linestyle=":", linewidth=1)

    # Annotations
    ax.text(65, -20, "65", ha="center", va="top", color="orange", fontsize=16)
    ax.text(55, -10, "55", ha="center", va="top", color="teal", fontsize=16)
    ax.text(750, -10, "750", ha="center", va="top", color="teal", fontsize=16)

    # Labels, ticks, grid, legend
    ax.set_xlabel("nb_agents", fontsize=18)
    ax.set_ylabel("Update (ms)", fontsize=18)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.set_yticks([0, 30, 50, 100, 150, 200])
    ax.tick_params(axis="both", labelsize=16)
    ax.grid(True, linestyle=":", linewidth=0.7)
    ax.legend(fontsize=16, title_fontsize=14, loc="upper left", bbox_to_anchor=(0.25, 0.85))

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "lrauv_uuvsim_lotusim_update_plot.eps")
    plt.savefig(out_path, format="eps")
    plt.show()
    print(f"✅ Plot saved to {out_path}")


# ==========================
# Main Execution
# ==========================


def main():
    # Load LRAUVSim CSV
    df_lrauv = load_lrauvsim_csv(LRAUVSIM_CSV)

    # Load LOTUSim data
    df_lotus = load_lotusim_data(LOTUSIM_DIR)

    # Save summary CSV
    save_lotusim_summary_csv(df_lotus)

    # Plot comparison
    plot_update_comparison(df_lrauv, df_lotus)


if __name__ == "__main__":
    main()
