#!/usr/bin/env python3
"""Offline analysis tool for PickPlaceTcpLogger CSV output.

Reads every pickplace_*.csv file in a given run directory (one file per
/pickplace call, produced by robot_task_manager/src/pickplace_server.cpp),
computes TCP tracking metrics per call and per stage, and renders plots.

Usage:
    python3 analyze_pickplace_tcp_logs.py --run-dir Report/executor_logs/run_YYYYMMDD_HHMMSS_PID

Does not require ROS2 to run — pure pandas/matplotlib post-processing of
already-written CSV files.
"""

import argparse
import sys
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401  (registers 3d projection)
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

REQUIRED_COLUMNS = [
    "time_sec", "stage", "row_type",
    "set_x", "set_y", "set_z",
    "actual_x", "actual_y", "actual_z",
    "error_pos_norm", "error_ori_rad",
    "status", "success", "message",
]

OPTIONAL_METADATA_COLUMNS = [
    "call_index", "velocity_scale",
    "pick_x", "pick_y", "pick_z",
    "place_x", "place_y", "place_z",
]


def rmse(values: pd.Series) -> float:
    values = values.dropna()
    if values.empty:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(values))))


def safe_stat(values: pd.Series, fn):
    values = values.dropna()
    if values.empty:
        return float("nan")
    return float(fn(values))


def load_csv(path: Path):
    """Read one pickplace_*.csv. Returns (df, warning_or_None)."""
    try:
        df = pd.read_csv(path, low_memory=False)
    except Exception as exc:  # noqa: BLE001 - deliberately broad, this is a best-effort tool
        return None, f"{path.name}: failed to read CSV ({exc})"

    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        return None, f"{path.name}: missing required columns {missing}"

    return df, None


def extract_call_index(df: pd.DataFrame, path: Path) -> str:
    if "call_index" in df.columns and not df["call_index"].dropna().empty:
        raw = df["call_index"].dropna().iloc[0]
        try:
            return f"{int(raw):04d}"
        except (TypeError, ValueError):
            return str(raw)
    # Fall back to parsing it out of the filename: pickplace_0001_....csv
    parts = path.stem.split("_")
    if len(parts) >= 2 and parts[1].isdigit():
        return parts[1]
    return path.stem


def metadata_value(df: pd.DataFrame, column: str):
    if column not in df.columns or df[column].dropna().empty:
        return float("nan")
    return df[column].dropna().iloc[0]


def build_summary_row(path: Path, df: pd.DataFrame) -> dict:
    call_index = extract_call_index(df, path)
    samples = df[df["row_type"] == "sample"]
    summary_rows = df[df["row_type"] == "summary"]

    if not summary_rows.empty:
        summary_row = summary_rows.iloc[-1]
        status = summary_row.get("status", "")
        success = summary_row.get("success", np.nan)
        message = summary_row.get("message", "")
        duration_sec = summary_row.get("time_sec", np.nan)
    else:
        # No summary row (e.g. node crashed mid-call, or file truncated) —
        # still report what we can instead of dropping the file.
        status = ""
        success = np.nan
        message = "no summary row found in file"
        duration_sec = safe_stat(df["time_sec"], np.max)

    error_pos = samples["error_pos_norm"]
    error_ori = samples["error_ori_rad"]
    final_error_pos_norm = float("nan")
    if not samples.empty:
        final_valid = samples["error_pos_norm"].dropna()
        if not final_valid.empty:
            final_error_pos_norm = float(final_valid.iloc[-1])

    return {
        "call_index": call_index,
        "csv_file": path.name,
        "success": success,
        "status": status,
        "message": message,
        "duration_sec": duration_sec,
        "row_count": len(df),
        "sample_count": len(samples),
        "max_error_pos_norm": safe_stat(error_pos, np.max),
        "mean_error_pos_norm": safe_stat(error_pos, np.mean),
        "rmse_error_pos_norm": rmse(error_pos),
        "final_error_pos_norm": final_error_pos_norm,
        "max_error_ori_rad": safe_stat(error_ori, np.max),
        "mean_error_ori_rad": safe_stat(error_ori, np.mean),
        "rmse_error_ori_rad": rmse(error_ori),
        "pick_x": metadata_value(df, "pick_x"),
        "pick_y": metadata_value(df, "pick_y"),
        "pick_z": metadata_value(df, "pick_z"),
        "place_x": metadata_value(df, "place_x"),
        "place_y": metadata_value(df, "place_y"),
        "place_z": metadata_value(df, "place_z"),
        "velocity_scale": metadata_value(df, "velocity_scale"),
    }


def build_stage_rows(path: Path, df: pd.DataFrame) -> list:
    call_index = extract_call_index(df, path)
    samples = df[df["row_type"] == "sample"].copy()
    if samples.empty:
        return []

    samples["stage"] = samples["stage"].fillna("(unknown)")
    rows = []
    for stage, group in samples.groupby("stage", sort=False):
        time_vals = group["time_sec"].dropna()
        stage_duration = (
            float(time_vals.max() - time_vals.min()) if len(time_vals) > 1 else 0.0
        )
        error_pos = group["error_pos_norm"]
        error_ori = group["error_ori_rad"]
        rows.append({
            "call_index": call_index,
            "csv_file": path.name,
            "stage": stage,
            "sample_count": len(group),
            "stage_duration_sec": stage_duration,
            "max_error_pos_norm": safe_stat(error_pos, np.max),
            "mean_error_pos_norm": safe_stat(error_pos, np.mean),
            "rmse_error_pos_norm": rmse(error_pos),
            "max_error_ori_rad": safe_stat(error_ori, np.max),
            "mean_error_ori_rad": safe_stat(error_ori, np.mean),
            "rmse_error_ori_rad": rmse(error_ori),
        })
    return rows


def plot_trajectory_3d(path: Path, df: pd.DataFrame, out_dir: Path, call_index: str):
    samples = df[df["row_type"] == "sample"]
    if samples.empty or samples[["actual_x", "actual_y", "actual_z"]].dropna().empty:
        return None

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    actual = samples[["actual_x", "actual_y", "actual_z"]].dropna()
    ax.plot(actual["actual_x"], actual["actual_y"], actual["actual_z"],
            label="TCP actual", color="tab:blue", linewidth=1.5)

    set_pts = samples[["set_x", "set_y", "set_z"]].dropna()
    if not set_pts.empty:
        ax.scatter(set_pts["set_x"], set_pts["set_y"], set_pts["set_z"],
                   label="TCP set (per-stage targets)", color="tab:red", s=8, alpha=0.5)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"PickPlace {call_index} — TCP trajectory (set vs actual)")
    ax.legend(loc="upper left")
    fig.tight_layout()

    out_path = out_dir / f"pickplace_{call_index}_trajectory_3d.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def plot_error_norm(path: Path, df: pd.DataFrame, out_dir: Path, call_index: str):
    samples = df[df["row_type"] == "sample"].dropna(subset=["time_sec"])
    if samples.empty:
        return None

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(samples["time_sec"], samples["error_pos_norm"], color="tab:orange", linewidth=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error_pos_norm (m)")
    ax.set_title(f"PickPlace {call_index} — TCP position error norm vs time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    out_path = out_dir / f"pickplace_{call_index}_error_norm.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def plot_error_xyz(path: Path, df: pd.DataFrame, out_dir: Path, call_index: str):
    samples = df[df["row_type"] == "sample"].dropna(subset=["time_sec"])
    if samples.empty:
        return None

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(samples["time_sec"], samples["error_x"], label="error_x", linewidth=1.0)
    ax.plot(samples["time_sec"], samples["error_y"], label="error_y", linewidth=1.0)
    ax.plot(samples["time_sec"], samples["error_z"], label="error_z", linewidth=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (m)")
    ax.set_title(f"PickPlace {call_index} — TCP position error components vs time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    out_path = out_dir / f"pickplace_{call_index}_error_xyz.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def plot_error_by_stage(path: Path, df: pd.DataFrame, out_dir: Path, call_index: str):
    samples = df[df["row_type"] == "sample"].copy()
    if samples.empty:
        return None
    samples["stage"] = samples["stage"].fillna("(unknown)")

    stages = list(dict.fromkeys(samples["stage"]))  # preserve first-seen order
    data = [samples.loc[samples["stage"] == s, "error_pos_norm"].dropna() for s in stages]
    data_nonempty = [d for d in data if len(d) > 0]
    stages_nonempty = [s for s, d in zip(stages, data) if len(d) > 0]
    if not data_nonempty:
        return None

    fig, ax = plt.subplots(figsize=(max(6, len(stages_nonempty) * 1.2), 4.5))
    # matplotlib < 3.9 uses `labels=`; newer versions renamed it to
    # `tick_labels=` (with `labels` deprecated). Try the new name first and
    # fall back for compatibility with whatever matplotlib is installed.
    try:
        ax.boxplot(data_nonempty, tick_labels=stages_nonempty, showfliers=False)
    except TypeError:
        ax.boxplot(data_nonempty, labels=stages_nonempty, showfliers=False)
    ax.set_ylabel("error_pos_norm (m)")
    ax.set_title(f"PickPlace {call_index} — TCP position error by stage")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()

    out_path = out_dir / f"pickplace_{call_index}_error_by_stage.png"
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-dir", required=True,
        help="Run directory containing pickplace_*.csv files "
             "(e.g. Report/executor_logs/run_YYYYMMDD_HHMMSS_PID)")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    if not run_dir.is_dir():
        print(f"ERROR: run dir not found: {run_dir}", file=sys.stderr)
        return 1

    csv_paths = sorted(run_dir.glob("pickplace_*.csv"))
    if not csv_paths:
        print(f"ERROR: no pickplace_*.csv files found in {run_dir}", file=sys.stderr)
        return 1

    analysis_dir = run_dir / "analysis"
    plots_dir = analysis_dir / "plots"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)

    warnings_list = []
    summary_rows = []
    stage_rows = []
    plotted_files = 0

    for path in csv_paths:
        df, warning = load_csv(path)
        if warning:
            warnings_list.append(warning)
            print(f"WARNING: {warning}", file=sys.stderr)
            continue

        try:
            summary_rows.append(build_summary_row(path, df))
        except Exception as exc:  # noqa: BLE001
            warnings_list.append(f"{path.name}: failed to compute summary ({exc})")
            print(f"WARNING: {path.name}: failed to compute summary ({exc})", file=sys.stderr)
            continue

        try:
            stage_rows.extend(build_stage_rows(path, df))
        except Exception as exc:  # noqa: BLE001
            warnings_list.append(f"{path.name}: failed to compute stage summary ({exc})")
            print(f"WARNING: {path.name}: failed to compute stage summary ({exc})", file=sys.stderr)

        call_index = extract_call_index(df, path)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            try:
                plot_trajectory_3d(path, df, plots_dir, call_index)
                plot_error_norm(path, df, plots_dir, call_index)
                plot_error_xyz(path, df, plots_dir, call_index)
                plot_error_by_stage(path, df, plots_dir, call_index)
                plotted_files += 1
            except Exception as exc:  # noqa: BLE001
                warnings_list.append(f"{path.name}: failed to plot ({exc})")
                print(f"WARNING: {path.name}: failed to plot ({exc})", file=sys.stderr)

    summary_path = analysis_dir / "pickplace_summary.csv"
    stage_summary_path = analysis_dir / "pickplace_stage_summary.csv"

    pd.DataFrame(summary_rows).to_csv(summary_path, index=False)
    pd.DataFrame(stage_rows).to_csv(stage_summary_path, index=False)

    print(f"Analyzed {len(summary_rows)}/{len(csv_paths)} CSV file(s) from {run_dir}")
    print(f"  {summary_path}")
    print(f"  {stage_summary_path}")
    print(f"  {plots_dir} ({plotted_files} file(s) plotted)")
    if warnings_list:
        print(f"  {len(warnings_list)} warning(s) — see stderr above")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
