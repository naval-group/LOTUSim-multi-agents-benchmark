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
@file statistics_accelerated_time.py
@author Naval Group

@brief Accelerated Time Simulation Performance Analysis (AI Training)

@note Analyzes RTF metrics from multiple simulators (LOTUSim, UUVSim, LRAUVSim)
and produces comparative visualizations.

Raw data and CSV summaries are saved in ./data/
Generated plots are saved in ./results/

@version 0.1
@date 2025-10-22
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from scipy.stats import skew, kurtosis

# ==========================
# Global Configuration
# ==========================
DATA_DIR = "data"
RESULTS_DIR = "results"
RAW_FILENAME = "simulation_1_raw_metrics.csv"
MIN_AGENTS = 1
MAX_AGENTS = 25

# Folders to process
FOLDERS_TO_PROCESS = {
    "LOTUSIM_LRAUV": os.path.join(DATA_DIR, "lotusim", "lrauv_sensors_bridge_gui"),
    "LOTUSIM_BlueROV": os.path.join(DATA_DIR, "lotusim", "bluerov_sensors_bridge"),
    "UUVSim": os.path.join(DATA_DIR, "uuvsim"),
    "LRAUVSim": None,  # Data comes only from CSV for this case
}

os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(RESULTS_DIR, exist_ok=True)

# ==========================
# Helper Functions
# ==========================


def safe_numeric(arr):
    """Convert a list to numeric values, discarding invalid entries (NaNs)."""
    numeric_arr = pd.to_numeric(arr, errors="coerce")
    return numeric_arr[~np.isnan(numeric_arr)]


def metric_evaluation(metrics, name):
    """Compute descriptive statistics for a numeric array."""
    stats = {
        "Metric": name,
        "Mean": round(np.mean(metrics), 4),
        "Median": round(np.median(metrics), 4),
        "Standard Deviation": round(np.std(metrics), 4),
        "Variance": round(np.var(metrics), 4),
        "Min": round(np.min(metrics), 4),
        "Max": round(np.max(metrics), 4),
    }

    if len(np.unique(metrics)) > 1:
        stats["Skewness"] = round(skew(metrics), 4)
        stats["Kurtosis"] = round(kurtosis(metrics), 4)
    else:
        stats["Skewness"] = 0.0
        stats["Kurtosis"] = 0.0

    return stats


# ==========================
# Data Processing Functions
# ==========================


def process_rtf_file(file_path):
    """
    Process a LOTUSim raw CSV file and return the mean RTF (normalized).
    Skips the first 350 warm-up steps.
    """
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()

    if "Real Time Factor (%)" not in df.columns:
        raise ValueError("Missing 'Real Time Factor (%)' column in CSV.")

    rtf_list = safe_numeric(df["Real Time Factor (%)"].tolist())
    if len(rtf_list) <= 350:
        raise ValueError("Insufficient RTF samples (<350).")

    rtf_list = rtf_list[350:]
    stats = metric_evaluation(rtf_list, "RTF")
    return stats["Mean"] / 100.0  # Convert % to factor


def collect_rtf_data(folder_name, summary_csv_name=None):
    """
    Collect RTF data from subfolders and save a summary CSV.
    Returns a DataFrame indexed by agent count.
    """
    rows = []

    for folder in os.listdir(folder_name):
        if not folder.isdigit():
            continue
        num_agents = int(folder)
        if not (MIN_AGENTS <= num_agents <= MAX_AGENTS):
            continue

        file_path = os.path.join(folder_name, folder, RAW_FILENAME)
        if not os.path.isfile(file_path):
            continue

        try:
            mean_rtf = process_rtf_file(file_path)
            rows.append({"Agents": num_agents, "Real Time Factor": mean_rtf})
        except Exception:
            continue

    df_summary = pd.DataFrame(rows).set_index("Agents").sort_index() if rows else pd.DataFrame()

    if not df_summary.empty:
        df_summary["Real Time Factor"] = df_summary["Real Time Factor"].round(2)

    if summary_csv_name:
        out_path = os.path.join(DATA_DIR, summary_csv_name)
        df_summary.to_csv(out_path)
        print(f"✅ Saved summary CSV: {out_path}")

    return df_summary


def extract_rtf_column(df):
    """Automatically detect the most likely RTF column in a CSV."""
    keywords = ["rtf", "real time"]
    candidates = [col for col in df.columns if any(k in col.lower() for k in keywords)]
    if not candidates:
        raise ValueError("No RTF-like column found in CSV.")
    return candidates[0]


def process_uuvsim_file(file_path):
    """Process a UUVSim CSV file and return the mean RTF (rows 100–900)."""
    df = pd.read_csv(file_path, on_bad_lines="skip")
    df.columns = df.columns.str.strip()
    rtf_col = extract_rtf_column(df)
    rtf_series = safe_numeric(df[rtf_col].iloc[100:900])
    if len(rtf_series) == 0:
        raise ValueError("No valid RTF samples in UUVSim CSV.")
    return round(np.mean(rtf_series), 4)


def collect_uuvsim_data(folder_name, summary_csv_name="uuvsim_rtf_plot_data.csv"):
    """Collect all UUVSim CSVs and save summary data."""
    rows = []

    for filename in os.listdir(folder_name):
        if not filename.startswith("benchmark_rtf_") or not filename.endswith(".csv"):
            continue
        try:
            num_agents = int(filename.replace(".csv", "").split("_")[2])
            if not (MIN_AGENTS <= num_agents <= MAX_AGENTS):
                continue
            mean_rtf = process_uuvsim_file(os.path.join(folder_name, filename))
            rows.append({"Agents": num_agents, "Real Time Factor": mean_rtf})
        except Exception:
            continue

    df_summary = pd.DataFrame(rows).set_index("Agents").sort_index() if rows else pd.DataFrame()
    if not df_summary.empty:
        df_summary["Real Time Factor"] = df_summary["Real Time Factor"].round(2)

    out_path = os.path.join(DATA_DIR, summary_csv_name)
    df_summary.to_csv(out_path)
    print(f"✅ Saved summary CSV: {out_path}")

    return df_summary


def load_data(label):
    """Load RTF summary CSV for a simulator."""
    csv_path = os.path.join(DATA_DIR, f"{label.lower()}_rtf_plot_data.csv")
    if os.path.isfile(csv_path):
        df = pd.read_csv(csv_path)
        return df.set_index("Agents").sort_index()
    else:
        print(f"⚠️ CSV not found for {label}: {csv_path}")
        return pd.DataFrame()


# ==========================
# Plotting Functions
# ==========================


def add_real_time_line(ax):
    """Add a horizontal line at RTF=1 to indicate real-time speed."""
    ax.axhline(y=1, color="red", linestyle=":", linewidth=1)
    ax.text(
        -0.05,
        1,
        "Real Time\n(1x speed)",
        color="red",
        fontsize=16,
        va="center",
        ha="right",
        transform=ax.get_yaxis_transform(),
    )


def configure_axes(ax, ymax):
    """Configure plot axes, ticks, labels, and grid."""
    ymin = 0
    ax.set_ylim(ymin, max(3, ymax + 1))

    yticks = ax.get_yticks()
    if 1 not in yticks:
        yticks = np.append(yticks, 1)
    ax.set_yticks(sorted(yticks))

    def y_formatter(val, _):
        return "" if val == 0 else str(int(val)) if val.is_integer() else str(val)

    ax.yaxis.set_major_formatter(mticker.FuncFormatter(y_formatter))

    ax.set_xlabel("nb_agents", fontsize=18)
    ax.set_ylabel("RTF (>1 = faster)", fontsize=18)
    ax.set_xticks(range(MIN_AGENTS, MAX_AGENTS + 1, 2))
    ax.set_xticklabels(range(MIN_AGENTS, MAX_AGENTS + 1, 2), fontsize=16)
    ax.tick_params(axis="y", labelsize=16)
    ax.grid(True, linestyle=":", linewidth=0.7)


def plot_curves(selected_labels, filename, custom_colors=None, custom_labels=None, title=""):
    """Plot RTF curves for selected simulators and save EPS."""
    fig, ax = plt.subplots(figsize=(12, 6))
    ymax = 0
    full_index = np.arange(MIN_AGENTS, MAX_AGENTS + 1)

    for label in selected_labels:
        df = load_data(label)
        if df.empty:
            continue
        df = df.reindex(full_index)
        color = custom_colors.get(label, "black") if custom_colors else "black"
        name = custom_labels.get(label, label) if custom_labels else label

        ax.plot(df.index, df["Real Time Factor"], marker="o", linestyle="-", color=color, label=name)
        if not df["Real Time Factor"].isna().all():
            ymax = max(ymax, df["Real Time Factor"].max())

    add_real_time_line(ax)
    configure_axes(ax, ymax)

    ax.legend(fontsize=16, title_fontsize=14, bbox_to_anchor=(0.6, 0.9))
    ax.set_title(title, fontsize=18)
    fig.tight_layout()

    out_path = os.path.join(RESULTS_DIR, filename)
    fig.savefig(out_path, format="eps")
    print(f"✅ Saved plot: {out_path}")

    plt.show()
    plt.close(fig)


# ==========================
# Custom Plot Configurations
# ==========================
LRAUV_COLORS = {"LOTUSIM_LRAUV": "teal", "LRAUVSim": "orange"}
LRAUV_LABELS = {"LOTUSIM_LRAUV": "LOTUSim", "LRAUVSim": "LRAUVSim"}

BLUEROV_COLORS = {"LOTUSIM_BlueROV": "teal", "UUVSim": "blue"}
BLUEROV_LABELS = {"LOTUSIM_BlueROV": "LOTUSim", "UUVSim": "UUVSim"}


# ==========================
# Main Execution
# ==========================
def main():
    # Generate summary CSVs for each simulator
    for label, folder in FOLDERS_TO_PROCESS.items():
        if folder is None or not os.path.isdir(folder):
            continue
        summary_csv_name = f"{label.lower()}_rtf_plot_data.csv"
        if label == "UUVSim":
            collect_uuvsim_data(folder, summary_csv_name)
        else:
            collect_rtf_data(folder, summary_csv_name)

    # Plot comparisons
    plot_curves(
        selected_labels=["LOTUSIM_BlueROV", "UUVSim"],
        filename="bluerov_ai_training_comparison.eps",
        custom_colors=BLUEROV_COLORS,
        custom_labels=BLUEROV_LABELS,
    )

    plot_curves(
        selected_labels=["LOTUSIM_LRAUV", "LRAUVSim"],
        filename="lrauv_ai_training_comparison.eps",
        custom_colors=LRAUV_COLORS,
        custom_labels=LRAUV_LABELS,
    )


if __name__ == "__main__":
    main()
