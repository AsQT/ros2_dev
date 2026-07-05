#!/usr/bin/env python3
"""Standalone GUI for previewing and plotting standardized robot log data."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np
import pandas as pd

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DEFAULT_LOG_ROOT = Path("/home/minhquang/ros2_dev/Log_robot_data")
SUPPORTED_FILES = {
    "summary.csv",
    "events.csv",
    "metadata.json",
    "rl_input_15d.csv",
    "rl_input_15d_pick.csv",
    "rl_input_15d_place.csv",
    "planning_rl.csv",
    "planning_moveit_from_rl.csv",
    "planning_pick.csv",
    "planning_place.csv",
    "trajectory_tracking.csv",
    "obstacle.csv",
    "object_obstacle.csv",
    "joint_tracking.csv",
    "tcp_tracking.csv",
}


@dataclass
class CsvLoadResult:
    path: Path
    df: pd.DataFrame | None
    warning: str = ""


@dataclass
class PlotResult:
    source: str
    outputs: list[Path]
    warnings: list[str]


def scan_log_root(log_root_dir: Path) -> dict[str, dict[str, dict[str, dict[str, list[str]]]]]:
    """Scan log root into runtime -> group -> action -> run -> calls."""
    tree: dict[str, dict[str, dict[str, dict[str, list[str]]]]] = {}
    root = Path(log_root_dir).expanduser()
    if not root.exists():
        return tree
    for runtime_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        if runtime_dir.name not in {"mock", "real"}:
            continue
        runtime_map = tree.setdefault(runtime_dir.name, {})
        for group_dir in sorted(p for p in runtime_dir.iterdir() if p.is_dir()):
            if group_dir.name not in {"rl", "baseline"}:
                continue
            group_map = runtime_map.setdefault(group_dir.name, {})
            for action_dir in sorted(p for p in group_dir.iterdir() if p.is_dir()):
                action_map = group_map.setdefault(action_dir.name, {})
                for run_dir in sorted(p for p in action_dir.iterdir() if p.is_dir()):
                    if not run_dir.name.startswith("run_") and not run_dir.name.startswith("campaign_"):
                        continue
                    calls = [
                        p.name for p in sorted(run_dir.iterdir())
                        if p.is_dir() and p.name.startswith("call_")
                    ]
                    if calls:
                        action_map[run_dir.name] = calls
    return tree


def get_runtime_modes(tree: dict) -> list[str]:
    return sorted(tree.keys())


def get_log_groups(tree: dict, runtime_mode: str) -> list[str]:
    return sorted(tree.get(runtime_mode, {}).keys())


def get_actions(tree: dict, runtime_mode: str, log_group: str) -> list[str]:
    return sorted(tree.get(runtime_mode, {}).get(log_group, {}).keys())


def get_runs(tree: dict, runtime_mode: str, log_group: str, action: str) -> list[str]:
    return sorted(tree.get(runtime_mode, {}).get(log_group, {}).get(action, {}).keys())


def get_calls(tree: dict, runtime_mode: str, log_group: str, action: str, run_id: str) -> list[str]:
    return sorted(tree.get(runtime_mode, {}).get(log_group, {}).get(action, {}).get(run_id, []))


def get_call_dir(
    log_root_dir: Path,
    runtime_mode: str,
    log_group: str,
    action: str,
    run_id: str,
    action_call_id: str,
) -> Path:
    return Path(log_root_dir).expanduser() / runtime_mode / log_group / action / run_id / action_call_id


def list_log_files(call_dir: Path) -> list[Path]:
    call = Path(call_dir)
    if not call.exists():
        return []
    files = []
    for path in sorted(call.iterdir()):
        if path.is_file() and (
            path.name in SUPPORTED_FILES or path.name.startswith("repeat_") and path.suffix == ".csv"
        ):
            files.append(path)
    return files


def load_csv_safe(path: Path) -> CsvLoadResult:
    try:
        if not Path(path).exists():
            return CsvLoadResult(path, None, "missing file")
        if Path(path).stat().st_size == 0:
            return CsvLoadResult(path, pd.DataFrame(), "empty file")
        df = pd.read_csv(path)
        if df.empty:
            return CsvLoadResult(path, df, "empty csv")
        return CsvLoadResult(path, df, "")
    except Exception as exc:  # noqa: BLE001 - GUI must never crash on bad logs.
        return CsvLoadResult(path, None, f"failed to read CSV: {exc}")


def load_metadata(path: Path) -> tuple[dict, str]:
    try:
        if not Path(path).exists():
            return {}, "missing file"
        with Path(path).open("r", encoding="utf-8") as f:
            return json.load(f), ""
    except Exception as exc:  # noqa: BLE001
        return {}, f"failed to read JSON: {exc}"


def detect_action_type(call_dir: Path, metadata: dict | None = None) -> str:
    if metadata and metadata.get("action_name"):
        return str(metadata["action_name"])
    parts = Path(call_dir).parts
    if len(parts) >= 2:
        return parts[-3] if parts[-2].startswith("run_") else parts[-1]
    return ""


def ensure_plots_dir(call_dir: Path) -> Path:
    out_dir = Path(call_dir) / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def save_figure(fig: plt.Figure, out_dir: Path, filename: str, overwrite: bool = True) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / filename
    if path.exists() and not overwrite:
        stem = path.stem
        suffix = path.suffix
        idx = 1
        while path.exists():
            path = out_dir / f"{stem}_{idx:03d}{suffix}"
            idx += 1
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def numeric_series(df: pd.DataFrame, column: str) -> pd.Series:
    if column not in df.columns:
        return pd.Series(dtype=float)
    return pd.to_numeric(df[column], errors="coerce")


def first_existing(df: pd.DataFrame, names: Iterable[str]) -> str | None:
    for name in names:
        if name in df.columns:
            return name
    return None


def require_columns(df: pd.DataFrame, columns: Iterable[str]) -> list[str]:
    return [c for c in columns if c not in df.columns]


def non_nan(values: Iterable[tuple[str, float]]) -> list[tuple[str, float]]:
    out = []
    for name, value in values:
        try:
            f = float(value)
        except Exception:
            continue
        if math.isfinite(f):
            out.append((name, f))
    return out


def plot_summary(df: pd.DataFrame, action_name: str, out_dir: Path, overwrite: bool = True) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    if df is None or df.empty:
        return PlotResult("summary.csv", outputs, ["summary.csv empty"])
    row = df.iloc[0]

    groups = [
        (
            "summary_timing.png",
            "Timing summary",
            [
                "planning_time_rl_s",
                "pick_planning_time_rl_s",
                "place_planning_time_rl_s",
                "planning_time_moveit_s",
                "planning_time_s",
                "execution_time_s",
                "total_time_s",
                "total_action_time_s",
            ],
        ),
        (
            "summary_path_error.png",
            "Path / error / clearance",
            [
                "rl_waypoint_count",
                "moveit_waypoint_count",
                "pick_rl_waypoint_count",
                "place_rl_waypoint_count",
                "path_length_rl_m",
                "path_length_moveit_m",
                "pick_path_length_m",
                "place_path_length_m",
                "total_path_length_m",
                "straight_line_distance_m",
                "path_efficiency",
                "final_position_error_m",
                "final_orientation_error_rad",
                "final_place_error_m",
                "min_obstacle_clearance_m",
                "avg_obstacle_clearance_m",
            ],
        ),
        (
            "summary_rl_metrics.png",
            "RL metrics",
            [
                "rl_final_distance_m",
                "rl_rollout_steps",
                "rl_reward_total",
                "rl_action_norm_mean",
                "rl_path_curvature",
                "rl_smoothness",
            ],
        ),
    ]

    for filename, title, columns in groups:
        values = non_nan((col, row[col]) for col in columns if col in df.columns)
        if not values:
            warnings.append(f"{filename}: no numeric supported columns")
            continue
        labels, nums = zip(*values)
        fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.7), 4.8))
        ax.bar(range(len(nums)), nums, color="#4c78a8")
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=35, ha="right")
        ax.set_title(f"{action_name} - {title}")
        ax.grid(axis="y", alpha=0.25)
        outputs.append(save_figure(fig, out_dir, filename, overwrite))

    text_items = []
    for col in ["success", "failed_stage", "failure_reason", "message", "action_name", "run_id", "action_call_id"]:
        if col in df.columns:
            text_items.append(f"{col}: {row[col]}")
    if text_items:
        fig, ax = plt.subplots(figsize=(10, max(3, len(text_items) * 0.35)))
        ax.axis("off")
        ax.text(0.01, 0.98, "\n".join(text_items), va="top", ha="left", family="monospace")
        ax.set_title(f"{action_name} - summary text")
        outputs.append(save_figure(fig, out_dir, "summary_status.png", overwrite))
    return PlotResult("summary.csv", outputs, warnings)


def plot_rl_input(df: pd.DataFrame, out_dir: Path, phase: str, overwrite: bool = True) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    missing = require_columns(df, ["input_index", "raw_value"])
    if missing:
        return PlotResult(phase, outputs, [f"{phase}: missing columns {missing}"])
    data = df.copy()
    data["input_index"] = pd.to_numeric(data["input_index"], errors="coerce")
    data["raw_value"] = pd.to_numeric(data["raw_value"], errors="coerce")
    data = data.sort_values("input_index")
    labels = data["input_name"].astype(str).tolist() if "input_name" in data.columns else data["input_index"].astype(str).tolist()
    fig, ax = plt.subplots(figsize=(max(8, len(data) * 0.55), 4.8))
    ax.bar(range(len(data)), data["raw_value"], color="#59a14f")
    ax.set_xticks(range(len(data)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_title(f"{phase} raw RL input")
    ax.set_ylabel("raw_value")
    ax.grid(axis="y", alpha=0.25)
    outputs.append(save_figure(fig, out_dir, f"{phase}_raw.png", overwrite))

    if "normalized_value" in data.columns:
        data["normalized_value"] = pd.to_numeric(data["normalized_value"], errors="coerce")
        if data["normalized_value"].notna().any():
            fig, ax = plt.subplots(figsize=(max(8, len(data) * 0.55), 4.8))
            ax.bar(range(len(data)), data["normalized_value"], color="#f28e2b")
            ax.set_xticks(range(len(data)))
            ax.set_xticklabels(labels, rotation=45, ha="right")
            ax.set_title(f"{phase} normalized RL input")
            ax.set_ylabel("normalized_value")
            ax.grid(axis="y", alpha=0.25)
            outputs.append(save_figure(fig, out_dir, f"{phase}_normalized.png", overwrite))
    return PlotResult(phase, outputs, warnings)


def obstacle_points(obstacle_df: pd.DataFrame | None) -> tuple[list[float], list[float], list[float]]:
    if obstacle_df is None or obstacle_df.empty:
        return [], [], []
    x_col = first_existing(obstacle_df, ["center_x", "obstacle_x"])
    y_col = first_existing(obstacle_df, ["center_y", "obstacle_y"])
    z_col = first_existing(obstacle_df, ["center_z", "obstacle_z"])
    if not x_col or not y_col or not z_col:
        return [], [], []
    return (
        numeric_series(obstacle_df, x_col).dropna().tolist(),
        numeric_series(obstacle_df, y_col).dropna().tolist(),
        numeric_series(obstacle_df, z_col).dropna().tolist(),
    )


def plot_planning_trajectory(
    df: pd.DataFrame,
    obstacle_df: pd.DataFrame | None,
    out_dir: Path,
    name: str,
    overwrite: bool = True,
) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    missing = require_columns(df, ["x", "y", "z"])
    if missing:
        return PlotResult(name, outputs, [f"{name}: missing columns {missing}"])
    data = df.copy()
    for col in ["x", "y", "z", "cumulative_path_length_m", "distance_to_target_m", "clearance_to_obstacle_m"]:
        if col in data.columns:
            data[col] = pd.to_numeric(data[col], errors="coerce")
    data = data.dropna(subset=["x", "y", "z"])
    if data.empty:
        return PlotResult(name, outputs, [f"{name}: no numeric xyz rows"])
    idx_col = first_existing(data, ["waypoint_index", "moveit_point_index", "planned_index"])
    if idx_col:
        data[idx_col] = pd.to_numeric(data[idx_col], errors="coerce")
    x_obs, y_obs, z_obs = obstacle_points(obstacle_df)

    fig = plt.figure(figsize=(7.5, 6.2))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["x"], data["y"], data["z"], marker="o", label=name)
    ax.scatter(data["x"].iloc[0], data["y"].iloc[0], data["z"].iloc[0], s=80, label="start")
    ax.scatter(data["x"].iloc[-1], data["y"].iloc[-1], data["z"].iloc[-1], s=80, label="target/final")
    if x_obs:
        ax.scatter(x_obs, y_obs, z_obs, marker="s", s=90, label="obstacle/object")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(f"{name} 3D trajectory")
    ax.legend()
    outputs.append(save_figure(fig, out_dir, f"{name}_3d.png", overwrite))

    for col, filename, title in [
        ("cumulative_path_length_m", f"{name}_cumulative_path.png", "Cumulative path length"),
        ("distance_to_target_m", f"{name}_distance_to_target.png", "Distance to target"),
        ("clearance_to_obstacle_m", f"{name}_clearance.png", "Clearance to obstacle"),
    ]:
        if col not in data.columns or not data[col].notna().any():
            continue
        x = data[idx_col] if idx_col else np.arange(len(data))
        fig, ax = plt.subplots(figsize=(8, 4.2))
        ax.plot(x, data[col], marker="o")
        ax.set_title(f"{name} - {title}")
        ax.set_xlabel(idx_col or "index")
        ax.set_ylabel(col)
        ax.grid(alpha=0.25)
        outputs.append(save_figure(fig, out_dir, filename, overwrite))

    joint_cols = [f"joint_{i}" for i in range(1, 7) if f"joint_{i}" in data.columns]
    if joint_cols:
        fig, ax = plt.subplots(figsize=(9, 5))
        x = data[idx_col] if idx_col else np.arange(len(data))
        for col in joint_cols:
            ax.plot(x, pd.to_numeric(data[col], errors="coerce"), label=col)
        ax.set_title(f"{name} joint trajectory")
        ax.set_xlabel(idx_col or "index")
        ax.set_ylabel("joint")
        ax.legend(ncol=3)
        ax.grid(alpha=0.25)
        outputs.append(save_figure(fig, out_dir, f"{name}_joints.png", overwrite))
    return PlotResult(name, outputs, warnings)


def plot_tracking(df: pd.DataFrame, out_dir: Path, name: str, overwrite: bool = True) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    if df is None or df.empty:
        return PlotResult(name, outputs, [f"{name}: empty tracking file"])
    time_col = first_existing(df, ["time_s", "time_sec", "t_rel_sec"])
    if not time_col:
        return PlotResult(name, outputs, [f"{name}: missing time column"])
    data = df.copy()
    data[time_col] = pd.to_numeric(data[time_col], errors="coerce")
    set_prefix = "set" if {"set_x", "set_y", "set_z"}.issubset(data.columns) else "target"
    has_set = {f"{set_prefix}_{axis}" for axis in "xyz"}.issubset(data.columns)
    has_actual = {"actual_x", "actual_y", "actual_z"}.issubset(data.columns)

    if has_set or has_actual:
        fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        for ax, axis in zip(axes, "xyz"):
            if has_set:
                ax.plot(data[time_col], pd.to_numeric(data[f"{set_prefix}_{axis}"], errors="coerce"), label=f"{set_prefix}_{axis}")
            if has_actual:
                ax.plot(data[time_col], pd.to_numeric(data[f"actual_{axis}"], errors="coerce"), label=f"actual_{axis}")
            ax.set_ylabel(axis)
            ax.grid(alpha=0.25)
            ax.legend(loc="best")
        axes[-1].set_xlabel("time_s")
        fig.suptitle(f"{name} TCP set/target vs actual")
        outputs.append(save_figure(fig, out_dir, f"{name}_xyz_tracking.png", overwrite))

    err_col = first_existing(df, ["position_error_m", "error_pos_norm"])
    ori_col = first_existing(df, ["orientation_error_rad", "error_ori_rad"])
    if err_col or ori_col:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        if err_col:
            ax.plot(data[time_col], pd.to_numeric(data[err_col], errors="coerce"), label=err_col)
        if ori_col:
            ax.plot(data[time_col], pd.to_numeric(data[ori_col], errors="coerce"), label=ori_col)
        ax.set_xlabel("time_s")
        ax.set_title(f"{name} tracking error")
        ax.grid(alpha=0.25)
        ax.legend()
        outputs.append(save_figure(fig, out_dir, f"{name}_error.png", overwrite))

    if has_actual:
        fig = plt.figure(figsize=(7.5, 6.2))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(
            pd.to_numeric(data["actual_x"], errors="coerce"),
            pd.to_numeric(data["actual_y"], errors="coerce"),
            pd.to_numeric(data["actual_z"], errors="coerce"),
            label="actual",
        )
        if has_set:
            ax.plot(
                pd.to_numeric(data[f"{set_prefix}_x"], errors="coerce"),
                pd.to_numeric(data[f"{set_prefix}_y"], errors="coerce"),
                pd.to_numeric(data[f"{set_prefix}_z"], errors="coerce"),
                label=set_prefix,
            )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title(f"{name} 3D trajectory")
        ax.legend()
        outputs.append(save_figure(fig, out_dir, f"{name}_3d.png", overwrite))

    if not outputs:
        warnings.append(f"{name}: no supported tracking columns")
    return PlotResult(name, outputs, warnings)


def plot_joint_tracking(
    df: pd.DataFrame,
    out_dir: Path,
    overwrite: bool = True,
    stem: str = "joint_tracking",
) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    label = f"{stem}.csv"
    tracking_png = "joint_tracking.png" if stem == "joint_tracking" else f"{stem}_tracking.png"
    error_png = "joint_error.png" if stem == "joint_tracking" else f"{stem}_error.png"
    if df is None or df.empty:
        return PlotResult(label, outputs, [f"{label} empty"])
    time_col = first_existing(df, ["time_s", "time_sec", "t_rel_sec"])
    if not time_col:
        return PlotResult(label, outputs, ["missing time column"])
    data = df.copy()
    data[time_col] = pd.to_numeric(data[time_col], errors="coerce")
    set_cols = [f"joint_{i}_set_rad" for i in range(1, 7)]
    actual_cols = [f"joint_{i}_actual_rad" for i in range(1, 7)]
    error_cols = [f"joint_{i}_error_rad" for i in range(1, 7)]

    if any(c in data.columns for c in set_cols + actual_cols):
        fig, axes = plt.subplots(6, 1, figsize=(10, 11), sharex=True)
        for i, ax in enumerate(axes, start=1):
            set_col = f"joint_{i}_set_rad"
            actual_col = f"joint_{i}_actual_rad"
            if set_col in data.columns:
                ax.plot(data[time_col], pd.to_numeric(data[set_col], errors="coerce"), label="set")
            if actual_col in data.columns:
                ax.plot(data[time_col], pd.to_numeric(data[actual_col], errors="coerce"), label="actual")
            ax.set_ylabel(f"j{i}")
            ax.grid(alpha=0.25)
            ax.legend(loc="best")
        axes[-1].set_xlabel("time_s")
        fig.suptitle(f"{stem}: joint set vs actual")
        outputs.append(save_figure(fig, out_dir, tracking_png, overwrite))

    if any(c in data.columns for c in error_cols) or "joint_error_norm_rad" in data.columns:
        fig, ax = plt.subplots(figsize=(10, 5))
        for col in error_cols:
            if col in data.columns:
                ax.plot(data[time_col], pd.to_numeric(data[col], errors="coerce"), label=col)
        if "joint_error_norm_rad" in data.columns:
            ax.plot(data[time_col], pd.to_numeric(data["joint_error_norm_rad"], errors="coerce"), label="joint_error_norm_rad", linewidth=2.5)
        ax.set_title(f"{stem}: joint error")
        ax.set_xlabel("time_s")
        ax.grid(alpha=0.25)
        ax.legend(ncol=2)
        outputs.append(save_figure(fig, out_dir, error_png, overwrite))

    if not outputs:
        warnings.append(f"{label}: no supported joint columns")
    return PlotResult(label, outputs, warnings)


def plot_repeatability(call_dir: Path, out_dir: Path, overwrite: bool = True) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    repeat_files = sorted(
        path for path in Path(call_dir).glob("repeat_*.csv")
        if not path.name.endswith("_joint.csv")
    )
    if not repeat_files:
        return PlotResult("repeatability", outputs, ["no repeat_*.csv files"])

    final_rows = []
    for idx, path in enumerate(repeat_files, start=1):
        loaded = load_csv_safe(path)
        if loaded.df is None or loaded.df.empty:
            warnings.append(f"{path.name}: {loaded.warning or 'empty'}")
            continue
        tracking_result = plot_tracking(loaded.df, out_dir, path.stem, overwrite)
        outputs.extend(tracking_result.outputs)
        warnings.extend(tracking_result.warnings)
        df = loaded.df.copy()
        for col in ["actual_x", "actual_y", "actual_z", "position_error_m", "orientation_error_rad"]:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors="coerce")
        if {"actual_x", "actual_y", "actual_z"}.issubset(df.columns):
            valid = df.dropna(subset=["actual_x", "actual_y", "actual_z"])
            if not valid.empty:
                row = valid.iloc[-1].to_dict()
                row["repeat_index"] = idx
                row["source_file"] = path.name
                final_rows.append(row)

    if final_rows:
        final_df = pd.DataFrame(final_rows)
        fig, ax = plt.subplots(figsize=(8, 4.5))
        if "position_error_m" in final_df.columns:
            ax.plot(final_df["repeat_index"], pd.to_numeric(final_df["position_error_m"], errors="coerce"), marker="o", label="position_error_m")
        if "orientation_error_rad" in final_df.columns:
            ax.plot(final_df["repeat_index"], pd.to_numeric(final_df["orientation_error_rad"], errors="coerce"), marker="o", label="orientation_error_rad")
        ax.set_xlabel("repeat_index")
        ax.set_title("Repeatability final error")
        ax.grid(alpha=0.25)
        ax.legend()
        outputs.append(save_figure(fig, out_dir, "repeatability_final_error.png", overwrite))

        fig = plt.figure(figsize=(7.5, 6.2))
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(final_df["actual_x"], final_df["actual_y"], final_df["actual_z"], s=70)
        mean_xyz = final_df[["actual_x", "actual_y", "actual_z"]].mean()
        std_xyz = final_df[["actual_x", "actual_y", "actual_z"]].std(ddof=0)
        ax.scatter([mean_xyz["actual_x"]], [mean_xyz["actual_y"]], [mean_xyz["actual_z"]], s=120, marker="x", label="mean")
        ax.set_title(
            "Repeatability final actual points\n"
            f"std=({std_xyz['actual_x']:.5f}, {std_xyz['actual_y']:.5f}, {std_xyz['actual_z']:.5f}) m"
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend()
        outputs.append(save_figure(fig, out_dir, "repeatability_scatter_3d.png", overwrite))
    else:
        warnings.append("repeatability: no final actual xyz rows")

    joint_files = sorted(Path(call_dir).glob("repeat_*_joint.csv"))
    joint_norm_rows = []
    for path in joint_files:
        loaded = load_csv_safe(path)
        if loaded.df is None or loaded.df.empty:
            warnings.append(f"{path.name}: {loaded.warning or 'empty'}")
            continue
        joint_result = plot_joint_tracking(loaded.df, out_dir, overwrite, path.stem)
        outputs.extend(joint_result.outputs)
        warnings.extend(joint_result.warnings)
        df = loaded.df.copy()
        if "joint_error_norm_rad" in df.columns:
            df["joint_error_norm_rad"] = pd.to_numeric(df["joint_error_norm_rad"], errors="coerce")
            if "repeat_index" in df.columns:
                df["repeat_index"] = pd.to_numeric(df["repeat_index"], errors="coerce")
            else:
                df["repeat_index"] = len(joint_norm_rows) + 1
            if "time_s" in df.columns:
                df["time_s"] = pd.to_numeric(df["time_s"], errors="coerce")
            joint_norm_rows.append(df[["repeat_index", "time_s", "joint_error_norm_rad"]].dropna(subset=["joint_error_norm_rad"]))

    if joint_norm_rows:
        norms = pd.concat(joint_norm_rows, ignore_index=True)
        if not norms.empty:
            fig, ax = plt.subplots(figsize=(10, 5))
            for repeat_index, group in norms.groupby("repeat_index"):
                x = group["time_s"] if "time_s" in group.columns else np.arange(len(group))
                ax.plot(x, group["joint_error_norm_rad"], marker="o", label=f"repeat {int(repeat_index)}")
            ax.set_title("Repeatability joint error norm")
            ax.set_xlabel("time_s")
            ax.set_ylabel("rad")
            ax.grid(alpha=0.25)
            ax.legend()
            outputs.append(save_figure(fig, out_dir, "repeatability_joint_error_norm.png", overwrite))
    return PlotResult("repeatability", outputs, warnings)


def plot_events(df: pd.DataFrame, out_dir: Path, overwrite: bool = True) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    if df is None or df.empty:
        return PlotResult("events.csv", outputs, ["events.csv empty"])
    t_col = first_existing(df, ["t_rel_sec", "time_s", "time_sec"])
    if not t_col:
        return PlotResult("events.csv", outputs, ["events.csv missing time column"])
    data = df.copy()
    data[t_col] = pd.to_numeric(data[t_col], errors="coerce")
    label_col = first_existing(data, ["stage", "event_type", "message"])
    fig, ax = plt.subplots(figsize=(10, max(3, min(10, len(data) * 0.28))))
    y = np.arange(len(data))
    ax.scatter(data[t_col], y)
    if label_col:
        for x_val, y_val, label in zip(data[t_col], y, data[label_col].astype(str)):
            if pd.notna(x_val):
                ax.text(x_val, y_val, label[:60], va="center", fontsize=8)
    ax.set_xlabel("t_rel_sec")
    ax.set_yticks([])
    ax.set_title("Event timeline")
    ax.grid(axis="x", alpha=0.25)
    outputs.append(save_figure(fig, out_dir, "events_timeline.png", overwrite))
    return PlotResult("events.csv", outputs, warnings)


def plot_object_table(df: pd.DataFrame, out_dir: Path, name: str, overwrite: bool = True) -> PlotResult:
    outputs: list[Path] = []
    warnings: list[str] = []
    if df is None or df.empty:
        return PlotResult(name, outputs, [f"{name}: empty"])
    x_col = first_existing(df, ["center_x", "x"])
    y_col = first_existing(df, ["center_y", "y"])
    z_col = first_existing(df, ["center_z", "z"])
    if x_col and y_col and z_col:
        fig = plt.figure(figsize=(7.5, 6.2))
        ax = fig.add_subplot(111, projection="3d")
        colors = df["object_type"].astype(str).tolist() if "object_type" in df.columns else None
        ax.scatter(
            pd.to_numeric(df[x_col], errors="coerce"),
            pd.to_numeric(df[y_col], errors="coerce"),
            pd.to_numeric(df[z_col], errors="coerce"),
            s=90,
        )
        if colors:
            for _, row in df.iterrows():
                ax.text(row[x_col], row[y_col], row[z_col], str(row.get("object_type", ""))[:20])
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title(name)
        outputs.append(save_figure(fig, out_dir, f"{Path(name).stem}_3d.png", overwrite))
    else:
        fig, ax = plt.subplots(figsize=(10, max(3, len(df) * 0.35)))
        ax.axis("off")
        ax.table(cellText=df.head(12).astype(str).values, colLabels=df.columns, loc="center")
        ax.set_title(name)
        outputs.append(save_figure(fig, out_dir, f"{Path(name).stem}_table.png", overwrite))
    return PlotResult(name, outputs, warnings)


def plot_file(call_dir: Path, file_path: Path, overwrite: bool = True) -> PlotResult:
    out_dir = ensure_plots_dir(call_dir)
    name = Path(file_path).name
    metadata, _ = load_metadata(Path(call_dir) / "metadata.json")
    action_name = detect_action_type(call_dir, metadata)
    obstacle_result = load_csv_safe(Path(call_dir) / "obstacle.csv")
    object_result = load_csv_safe(Path(call_dir) / "object_obstacle.csv")
    obstacle_df = obstacle_result.df if obstacle_result.df is not None else object_result.df

    if name == "metadata.json":
        metadata, warning = load_metadata(file_path)
        if warning:
            return PlotResult(name, [], [warning])
        fig, ax = plt.subplots(figsize=(10, max(3, len(metadata) * 0.32)))
        ax.axis("off")
        ax.text(0.01, 0.98, json.dumps(metadata, indent=2, ensure_ascii=False), va="top", family="monospace")
        return PlotResult(name, [save_figure(fig, out_dir, "metadata.png", overwrite)], [])

    loaded = load_csv_safe(file_path)
    if loaded.df is None:
        return PlotResult(name, [], [loaded.warning])
    if loaded.warning:
        return PlotResult(name, [], [loaded.warning])
    df = loaded.df

    if name == "summary.csv":
        return plot_summary(df, action_name, out_dir, overwrite)
    if name.startswith("rl_input_15d"):
        phase = Path(name).stem
        return plot_rl_input(df, out_dir, phase, overwrite)
    if name in {"planning_rl.csv", "planning_moveit_from_rl.csv", "planning_pick.csv", "planning_place.csv"}:
        return plot_planning_trajectory(df, obstacle_df, out_dir, Path(name).stem, overwrite)
    if name.endswith("_joint.csv"):
        return plot_joint_tracking(df, out_dir, overwrite, Path(name).stem)
    if name in {"trajectory_tracking.csv", "tcp_tracking.csv"} or name.startswith("repeat_"):
        return plot_tracking(df, out_dir, Path(name).stem, overwrite)
    if name == "joint_tracking.csv":
        return plot_joint_tracking(df, out_dir, overwrite)
    if name == "events.csv":
        return plot_events(df, out_dir, overwrite)
    if name in {"obstacle.csv", "object_obstacle.csv"}:
        return plot_object_table(df, out_dir, name, overwrite)
    return PlotResult(name, [], [f"{name}: no plot handler"])


def plot_repeatability_if_available(call_dir: Path, overwrite: bool = True) -> PlotResult:
    return plot_repeatability(call_dir, ensure_plots_dir(call_dir), overwrite)


def plot_all_available(call_dir: Path, overwrite: bool = True) -> list[PlotResult]:
    results = []
    for path in list_log_files(call_dir):
        results.append(plot_file(call_dir, path, overwrite))
    repeat_result = plot_repeatability_if_available(call_dir, overwrite)
    if repeat_result.outputs or repeat_result.warnings != ["no repeat_*.csv files"]:
        results.append(repeat_result)
    return results


def preview_file(path: Path) -> str:
    if not path.exists():
        return "Missing file"
    if path.suffix == ".json":
        data, warning = load_metadata(path)
        if warning:
            return warning
        return json.dumps(data, indent=2, ensure_ascii=False)
    loaded = load_csv_safe(path)
    if loaded.df is None:
        return loaded.warning
    df = loaded.df
    lines = [
        f"File: {path}",
        f"Rows: {len(df)}",
        f"Columns: {len(df.columns)}",
        "Column names:",
        ", ".join(df.columns.astype(str)),
    ]
    if loaded.warning:
        lines.append(f"Warning: {loaded.warning}")
    if not df.empty:
        lines.extend(["", "First 10 rows:", df.head(10).to_string(index=False)])
    return "\n".join(lines)


class LogDataViewerApp:
    def __init__(self, root, log_root_dir: Path = DEFAULT_LOG_ROOT) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.root.title("Robot Log Data Viewer")
        self.log_root_var = tk.StringVar(value=str(log_root_dir))
        self.runtime_var = tk.StringVar()
        self.group_var = tk.StringVar()
        self.action_var = tk.StringVar()
        self.run_var = tk.StringVar()
        self.call_var = tk.StringVar()
        self.overwrite_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Ready")
        self.tree: dict = {}
        self.file_paths: list[Path] = []

        self._build_ui()
        self.refresh_tree()

    def _build_ui(self) -> None:
        tk = self.tk
        ttk = self.ttk
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(3, weight=1)

        ttk.Label(main, text="Log root").grid(row=0, column=0, sticky="w")
        ttk.Entry(main, textvariable=self.log_root_var).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(main, text="Browse", command=self.choose_root).grid(row=0, column=2, padx=2)
        ttk.Button(main, text="Refresh", command=self.refresh_tree).grid(row=0, column=3, padx=2)
        ttk.Button(main, text="Open log root", command=lambda: self.open_path(Path(self.log_root_var.get()))).grid(row=0, column=4, padx=2)

        selectors = ttk.Frame(main)
        selectors.grid(row=1, column=0, columnspan=5, sticky="ew", pady=6)
        for i in range(6):
            selectors.columnconfigure(i, weight=1)
        self.runtime_cb = self._combo(selectors, "runtime_mode", self.runtime_var, 0, self.on_runtime)
        self.group_cb = self._combo(selectors, "log_group", self.group_var, 1, self.on_group)
        self.action_cb = self._combo(selectors, "action_name", self.action_var, 2, self.on_action)
        self.run_cb = self._combo(selectors, "run_id", self.run_var, 3, self.on_run)
        self.call_cb = self._combo(selectors, "action_call_id", self.call_var, 4, self.on_call)
        ttk.Checkbutton(selectors, text="Overwrite plots", variable=self.overwrite_var).grid(row=1, column=5, sticky="w", padx=4)

        buttons = ttk.Frame(main)
        buttons.grid(row=2, column=0, columnspan=5, sticky="ew")
        ttk.Button(buttons, text="Open selected call folder", command=self.open_call_dir).pack(side="left", padx=2)
        ttk.Button(buttons, text="Plot selected", command=self.plot_selected).pack(side="left", padx=2)
        ttk.Button(buttons, text="Plot all available", command=self.plot_all).pack(side="left", padx=2)
        ttk.Button(buttons, text="Save plots", command=self.plot_selected).pack(side="left", padx=2)

        panes = ttk.PanedWindow(main, orient="horizontal")
        panes.grid(row=3, column=0, columnspan=5, sticky="nsew", pady=6)
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=1)
        panes.add(right, weight=3)

        ttk.Label(left, text="Log files").pack(anchor="w")
        self.file_list = tk.Listbox(left, selectmode=tk.EXTENDED, height=18)
        self.file_list.pack(fill="both", expand=True)
        self.file_list.bind("<<ListboxSelect>>", lambda _event: self.preview_selected())

        ttk.Label(right, text="Preview").pack(anchor="w")
        self.preview = tk.Text(right, wrap="none", height=24)
        self.preview.pack(fill="both", expand=True)

        ttk.Label(main, textvariable=self.status_var).grid(row=4, column=0, columnspan=5, sticky="ew")

    def _combo(self, parent, label: str, variable, column: int, callback: Callable) -> object:
        ttk = self.ttk
        ttk.Label(parent, text=label).grid(row=0, column=column, sticky="w")
        cb = ttk.Combobox(parent, textvariable=variable, state="readonly", width=18)
        cb.grid(row=1, column=column, sticky="ew", padx=2)
        cb.bind("<<ComboboxSelected>>", lambda _event: callback())
        return cb

    def set_combo_values(self, cb, values: list[str], var) -> None:
        cb["values"] = values
        var.set(values[0] if values else "")

    def refresh_tree(self) -> None:
        root = Path(self.log_root_var.get()).expanduser()
        if not root.exists():
            self.status_var.set(f"Log root does not exist: {root}")
        self.tree = scan_log_root(root)
        self.set_combo_values(self.runtime_cb, get_runtime_modes(self.tree), self.runtime_var)
        self.on_runtime()
        self.status_var.set(f"Scanned {root}")

    def on_runtime(self) -> None:
        self.set_combo_values(self.group_cb, get_log_groups(self.tree, self.runtime_var.get()), self.group_var)
        self.on_group()

    def on_group(self) -> None:
        self.set_combo_values(
            self.action_cb,
            get_actions(self.tree, self.runtime_var.get(), self.group_var.get()),
            self.action_var,
        )
        self.on_action()

    def on_action(self) -> None:
        self.set_combo_values(
            self.run_cb,
            get_runs(self.tree, self.runtime_var.get(), self.group_var.get(), self.action_var.get()),
            self.run_var,
        )
        self.on_run()

    def on_run(self) -> None:
        self.set_combo_values(
            self.call_cb,
            get_calls(
                self.tree,
                self.runtime_var.get(),
                self.group_var.get(),
                self.action_var.get(),
                self.run_var.get(),
            ),
            self.call_var,
        )
        self.on_call()

    def selected_call_dir(self) -> Path:
        return get_call_dir(
            Path(self.log_root_var.get()),
            self.runtime_var.get(),
            self.group_var.get(),
            self.action_var.get(),
            self.run_var.get(),
            self.call_var.get(),
        )

    def on_call(self) -> None:
        self.file_list.delete(0, self.tk.END)
        self.preview.delete("1.0", self.tk.END)
        call_dir = self.selected_call_dir()
        self.file_paths = list_log_files(call_dir)
        for path in self.file_paths:
            self.file_list.insert(self.tk.END, path.name)
        if self.file_paths:
            self.file_list.selection_set(0)
            self.preview_selected()

    def preview_selected(self) -> None:
        selection = self.file_list.curselection()
        self.preview.delete("1.0", self.tk.END)
        if not selection:
            return
        path = self.file_paths[selection[0]]
        self.preview.insert("1.0", preview_file(path))

    def selected_files(self) -> list[Path]:
        selection = self.file_list.curselection()
        return [self.file_paths[i] for i in selection]

    def choose_root(self) -> None:
        from tkinter import filedialog

        chosen = filedialog.askdirectory(initialdir=str(Path(self.log_root_var.get()).expanduser()))
        if chosen:
            self.log_root_var.set(chosen)
            self.refresh_tree()

    def open_path(self, path: Path) -> None:
        try:
            subprocess.Popen(["xdg-open", str(path)])
        except Exception as exc:  # noqa: BLE001
            self.status_var.set(f"Failed to open {path}: {exc}")

    def open_call_dir(self) -> None:
        self.open_path(self.selected_call_dir())

    def show_results(self, results: list[PlotResult]) -> None:
        outputs = [str(p) for result in results for p in result.outputs]
        warnings = [w for result in results for w in result.warnings]
        lines = []
        if outputs:
            lines.append("Saved plots:")
            lines.extend(outputs)
        if warnings:
            lines.append("")
            lines.append("Warnings:")
            lines.extend(warnings)
        if not lines:
            lines.append("No plots generated.")
        self.preview.delete("1.0", self.tk.END)
        self.preview.insert("1.0", "\n".join(lines))
        self.status_var.set(f"Saved {len(outputs)} plot(s); {len(warnings)} warning(s)")

    def plot_selected(self) -> None:
        call_dir = self.selected_call_dir()
        results = [plot_file(call_dir, path, self.overwrite_var.get()) for path in self.selected_files()]
        self.show_results(results)

    def plot_all(self) -> None:
        self.show_results(plot_all_available(self.selected_call_dir(), self.overwrite_var.get()))


def create_sample_log(root: Path) -> Path:
    if root.exists():
        shutil.rmtree(root)

    run_id = "run_20260704_010203"
    call_id = "call_0001"
    start_iso = "2026-07-04T01:02:03.000"
    end_iso = "2026-07-04T01:02:05.000"
    n = 15
    t = np.linspace(0, 1.4, n)
    xs = np.linspace(0.30, 0.45, n)
    ys = 0.03 * np.sin(np.linspace(0, np.pi, n))
    zs = np.linspace(0.20, 0.30, n)
    step = np.r_[0.0, np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2 + np.diff(zs) ** 2)]

    def call_dir(group: str, action: str) -> Path:
        path = root / "mock" / group / action / run_id / call_id
        path.mkdir(parents=True, exist_ok=True)
        return path

    def write_metadata(path: Path, group: str, action: str, planner_type: str) -> None:
        (path / "metadata.json").write_text(
            json.dumps(
                {
                    "runtime_mode": "mock",
                    "log_group": group,
                    "action_name": action,
                    "run_id": run_id,
                    "action_call_id": call_id,
                    "parent_action_call_id": "",
                    "goal_uuid": "",
                    "robot_model": "sample_robot",
                    "base_frame": "base_link",
                    "tcp_frame": "tcp_link",
                    "hardware_backend": "mock",
                    "vision_source": "sample",
                    "planner_type": planner_type,
                    "model_path": "" if planner_type != "rl" else "/tmp/sample_policy.pt",
                    "created_by_node": "log_data_viewer_self_test",
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

    def summary_base(group: str, action: str) -> dict[str, object]:
        return {
            "runtime_mode": "mock",
            "log_group": group,
            "action_name": action,
            "run_id": run_id,
            "action_call_id": call_id,
            "parent_action_call_id": "",
            "goal_uuid": "",
            "start_timestamp_iso": start_iso,
            "end_timestamp_iso": end_iso,
            "execute_requested": True,
            "success": True,
            "failed_stage": "",
            "failure_reason": "",
            "message": "sample done",
            "total_time_s": 2.0,
        }

    def write_events(path: Path) -> None:
        pd.DataFrame(
            {
                "timestamp_iso": [start_iso, end_iso],
                "t_rel_sec": [0.0, 2.0],
                "stage": ["start", "done"],
                "event_type": ["action_start", "action_result"],
                "success": ["", "true"],
                "message": ["sample start", "sample done"],
            }
        ).to_csv(path / "events.csv", index=False)

    def rl_input(path: Path, name: str, phase: str) -> None:
        pd.DataFrame(
            {
                "phase": [phase] * 15,
                "input_index": list(range(15)),
                "input_name": [f"obs_{i}" for i in range(15)],
                "raw_value": np.linspace(-1, 1, 15),
                "normalized_value": np.linspace(-0.5, 0.5, 15),
                "unit": ["m"] * 15,
                "source": ["sample_observation_order"] * 15,
            }
        ).to_csv(path / name, index=False)

    def planning(path: Path, name: str, x_offset: float = 0.0) -> None:
        pd.DataFrame(
            {
                "waypoint_index": range(n),
                "timestamp_iso": [start_iso] * n,
                "x": xs + x_offset,
                "y": ys,
                "z": zs,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
                "step_distance_m": step,
                "cumulative_path_length_m": np.cumsum(step),
                "distance_to_target_m": np.linspace(0.28, 0.0, n),
                "clearance_to_obstacle_m": np.linspace(0.12, 0.08, n),
                "workspace_valid": True,
            }
        ).to_csv(path / name, index=False)

    def tracking(path: Path, stages: list[str], x_offset: float = 0.0) -> None:
        repeated_stages = [stages[min(i * len(stages) // n, len(stages) - 1)] for i in range(n)]
        pd.DataFrame(
            {
                "time_s": t,
                "stage": repeated_stages,
                "target_x": xs + x_offset,
                "target_y": ys,
                "target_z": zs,
                "target_qx": 0.0,
                "target_qy": 0.0,
                "target_qz": 0.0,
                "target_qw": 1.0,
                "actual_x": xs + x_offset + 0.002,
                "actual_y": ys - 0.001,
                "actual_z": zs + 0.001,
                "actual_qx": 0.0,
                "actual_qy": 0.0,
                "actual_qz": 0.0,
                "actual_qw": 1.0,
                "position_error_m": np.linspace(0.006, 0.003, n),
                "orientation_error_rad": np.linspace(0.03, 0.01, n),
            }
        ).to_csv(path / "trajectory_tracking.csv", index=False)

    move_rl = call_dir("rl", "move_pose_rl")
    write_metadata(move_rl, "rl", "move_pose_rl", "rl")
    pd.DataFrame(
        [
            {
                **summary_base("rl", "move_pose_rl"),
                "planning_time_rl_s": 0.4,
                "planning_time_moveit_s": 0.18,
                "execution_time_s": 1.3,
                "rl_waypoint_count": 15,
                "moveit_waypoint_count": 18,
                "path_length_rl_m": 0.32,
                "path_length_moveit_m": 0.34,
                "straight_line_distance_m": 0.28,
                "path_efficiency": 0.875,
                "target_x": 0.45,
                "target_y": 0.0,
                "target_z": 0.30,
                "start_tcp_x": 0.30,
                "start_tcp_y": 0.0,
                "start_tcp_z": 0.20,
                "final_tcp_x": 0.452,
                "final_tcp_y": -0.001,
                "final_tcp_z": 0.301,
                "final_position_error_m": 0.004,
                "final_orientation_error_rad": 0.02,
                "min_obstacle_clearance_m": 0.08,
                "avg_obstacle_clearance_m": 0.11,
                "rl_reward_total": 12.3,
            }
        ]
    ).to_csv(move_rl / "summary.csv", index=False)
    rl_input(move_rl, "rl_input_15d.csv", "plan")
    planning(move_rl, "planning_rl.csv")
    planning(move_rl, "planning_moveit_from_rl.csv", 0.004)
    pd.DataFrame(
        {
            "obstacle_id": ["box_1"],
            "source": ["sample"],
            "frame_id": ["base_link"],
            "center_x": [0.38],
            "center_y": [0.02],
            "center_z": [0.25],
            "size_x": [0.05],
            "size_y": [0.05],
            "size_z": [0.10],
            "qx": [0.0],
            "qy": [0.0],
            "qz": [0.0],
            "qw": [1.0],
            "safety_margin_m": [0.03],
        }
    ).to_csv(move_rl / "obstacle.csv", index=False)
    tracking(move_rl, ["execute"])
    write_events(move_rl)

    pick_rl = call_dir("rl", "pick_place_rl")
    write_metadata(pick_rl, "rl", "pick_place_rl", "rl")
    pd.DataFrame(
        [
            {
                **summary_base("rl", "pick_place_rl"),
                "pick_planning_time_rl_s": 0.34,
                "place_planning_time_rl_s": 0.37,
                "execution_time_s": 1.2,
                "pick_rl_waypoint_count": 15,
                "place_rl_waypoint_count": 15,
                "pick_path_length_m": 0.22,
                "place_path_length_m": 0.27,
                "final_place_error_m": 0.006,
                "min_obstacle_clearance_m": 0.07,
            }
        ]
    ).to_csv(pick_rl / "summary.csv", index=False)
    rl_input(pick_rl, "rl_input_15d_pick.csv", "pick")
    rl_input(pick_rl, "rl_input_15d_place.csv", "place")
    planning(pick_rl, "planning_pick.csv")
    planning(pick_rl, "planning_place.csv", 0.05)
    tracking(pick_rl, ["pre_pick", "approach_pick", "grasp", "lift", "approach_place", "release"], 0.02)
    pd.DataFrame(
        {
            "object_id": ["wood_1", "obstacle_1", "place_target"],
            "object_type": ["wood", "obstacle", "place_target"],
            "source": ["sample", "sample", "sample"],
            "frame_id": ["base_link", "base_link", "base_link"],
            "center_x": [0.36, 0.40, 0.48],
            "center_y": [0.02, -0.03, 0.04],
            "center_z": [0.18, 0.24, 0.18],
            "size_x": [0.04, 0.05, 0.04],
            "size_y": [0.04, 0.05, 0.04],
            "size_z": [0.08, 0.10, 0.02],
            "qx": [0.0, 0.0, 0.0],
            "qy": [0.0, 0.0, 0.0],
            "qz": [0.0, 0.0, 0.0],
            "qw": [1.0, 1.0, 1.0],
            "safety_margin_m": [0.02, 0.03, 0.02],
        }
    ).to_csv(pick_rl / "object_obstacle.csv", index=False)
    write_events(pick_rl)

    baseline = call_dir("baseline", "move_checkerboard")
    write_metadata(baseline, "baseline", "move_checkerboard", "baseline")
    joint_data = {"time_s": t, "stage": ["sample"] * len(t)}
    for i in range(1, 7):
        joint_data[f"joint_{i}_set_rad"] = np.sin(t + i * 0.1)
        joint_data[f"joint_{i}_actual_rad"] = np.sin(t + i * 0.1) + 0.01
        joint_data[f"joint_{i}_error_rad"] = 0.01
    joint_data["joint_error_norm_rad"] = 0.024
    pd.DataFrame(joint_data).to_csv(baseline / "joint_tracking.csv", index=False)
    tracking(baseline, ["sample"])
    (baseline / "trajectory_tracking.csv").rename(baseline / "tcp_tracking.csv")
    pd.DataFrame(
        [
            {
                **summary_base("baseline", "move_checkerboard"),
                "waypoint_count": 20,
                "execution_time_s": 1.0,
                "mean_joint_error_norm": 0.024,
                "max_joint_error_norm": 0.03,
                "rmse_joint_error_norm": 0.025,
                "mean_tcp_position_error_m": 0.004,
                "max_tcp_position_error_m": 0.006,
                "rmse_tcp_position_error_m": 0.0045,
            }
        ]
    ).to_csv(baseline / "summary.csv", index=False)
    write_events(baseline)

    repeat = call_dir("baseline", "repeatability_test")
    write_metadata(repeat, "baseline", "repeatability_test", "baseline")
    for r in range(1, 4):
        pd.DataFrame(
            {
                "time_s": t,
                "stage": [f"loop_{r}_measure"] * len(t),
                "target_x": 0.4,
                "target_y": 0.0,
                "target_z": 0.2,
                "target_qx": 0.0,
                "target_qy": 0.0,
                "target_qz": 0.0,
                "target_qw": 1.0,
                "actual_x": 0.4 + r * 0.001 + t * 0.0001,
                "actual_y": 0.0 + r * 0.0005,
                "actual_z": 0.2 - r * 0.0003,
                "actual_qx": 0.0,
                "actual_qy": 0.0,
                "actual_qz": 0.0,
                "actual_qw": 1.0,
                "position_error_m": 0.002 + r * 0.0002,
                "orientation_error_rad": 0.01 + r * 0.001,
            }
        ).to_csv(repeat / f"repeat_{r:04d}.csv", index=False)
    pd.DataFrame(
        [
            {
                **summary_base("baseline", "repeatability_test"),
                "axis": "x",
                "repeat_count": 3,
                "offset_m": 0.01,
                "success_count": 3,
                "failed_count": 0,
                "mean_position_error_m": 0.0024,
                "max_position_error_m": 0.0026,
                "rmse_position_error_m": 0.00245,
                "repeatability_position_std_m": 0.0002,
            }
        ]
    ).to_csv(repeat / "summary.csv", index=False)
    write_events(repeat)

    pick_base = call_dir("baseline", "pick_place")
    write_metadata(pick_base, "baseline", "pick_place", "baseline")
    pd.DataFrame(
        [
            {
                **summary_base("baseline", "pick_place"),
                "planning_time_s": 0.22,
                "execution_time_s": 1.4,
                "waypoint_count": 18,
                "total_path_length_m": 0.36,
                "final_position_error_m": 0.005,
                "final_orientation_error_rad": 0.015,
            }
        ]
    ).to_csv(pick_base / "summary.csv", index=False)
    tracking(pick_base, ["approach_pick", "grasp", "approach_place", "release"], 0.04)
    write_events(pick_base)

    return root


def run_self_test() -> int:
    root = create_sample_log(Path("/tmp/log_data_viewer_test"))
    tree = scan_log_root(root)
    if "mock" not in tree:
        print("self-test failed: mock branch not found", file=sys.stderr)
        return 1
    cases = [
        root / "mock" / "rl" / "move_pose_rl" / "run_20260704_010203" / "call_0001",
        root / "mock" / "rl" / "pick_place_rl" / "run_20260704_010203" / "call_0001",
        root / "mock" / "baseline" / "move_checkerboard" / "run_20260704_010203" / "call_0001",
        root / "mock" / "baseline" / "repeatability_test" / "run_20260704_010203" / "call_0001",
        root / "mock" / "baseline" / "pick_place" / "run_20260704_010203" / "call_0001",
    ]
    all_outputs: list[Path] = []
    all_warnings: list[str] = []
    for call in cases:
        results = plot_all_available(call, overwrite=True)
        all_outputs.extend(p for result in results for p in result.outputs)
        all_warnings.extend(w for result in results for w in result.warnings)
    required = [
        cases[0] / "plots" / "summary_timing.png",
        cases[0] / "plots" / "rl_input_15d_raw.png",
        cases[0] / "plots" / "planning_rl_3d.png",
        cases[0] / "plots" / "trajectory_tracking_xyz_tracking.png",
        cases[1] / "plots" / "rl_input_15d_pick_raw.png",
        cases[1] / "plots" / "planning_pick_3d.png",
        cases[1] / "plots" / "object_obstacle_3d.png",
        cases[2] / "plots" / "joint_tracking.png",
        cases[2] / "plots" / "tcp_tracking_xyz_tracking.png",
        cases[3] / "plots" / "repeatability_final_error.png",
        cases[4] / "plots" / "trajectory_tracking_xyz_tracking.png",
    ]
    missing = [p for p in required if not p.exists()]
    print(f"Sample root: {root}")
    print(f"Generated plots: {len(all_outputs)}")
    for path in all_outputs[:20]:
        print(path)
    if all_warnings:
        print("Warnings:")
        for warning in all_warnings:
            print(f"- {warning}")
    if missing:
        print("Missing required plots:", file=sys.stderr)
        for path in missing:
            print(path, file=sys.stderr)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-root", default=str(DEFAULT_LOG_ROOT), help="Log root directory")
    parser.add_argument("--self-test", action="store_true", help="Create sample logs under /tmp and plot without GUI")
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()

    try:
        import tkinter as tk
    except Exception as exc:  # noqa: BLE001
        print(f"tkinter is not available: {exc}", file=sys.stderr)
        print("Install it with: sudo apt install python3-tk", file=sys.stderr)
        return 2
    root = tk.Tk()
    LogDataViewerApp(root, Path(args.log_root))
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
