#!/usr/bin/env python3
"""plot_action_log.py — verification plots for an action evaluation log.

codex.md §8: given a ``call_XXXX/`` directory (or a ``run_.../`` / group
directory to process recursively), auto-detect the action/group from
``metadata.json``, read whatever CSVs are actually present, and render quick
verification plots into ``<call>/plots/``. It is deliberately resilient:

* It NEVER raises out of ``plot_call`` — every plot is wrapped, and any
  failure is recorded in ``plots/plot_status.json`` instead of aborting.
* If matplotlib (or pandas-free numpy parsing) is unavailable, it records a
  ``skipped`` status and exits 0, so a caller (e.g. after ``finish()``) is
  never turned into an action failure.
* Missing/empty CSVs are skipped with a reason, not treated as errors.

Usage:
    plot_action_log.py <call_dir | run_dir | any parent dir> [--quiet]

Exit code is always 0 (best-effort verification tool).
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
import traceback
from typing import Dict, List, Optional, Tuple

# --- Soft matplotlib import (headless). Never fatal. ------------------------
_MPL_OK = True
_MPL_ERR = ""
try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402
except Exception as exc:  # pragma: no cover - environment dependent
    _MPL_OK = False
    _MPL_ERR = f"matplotlib unavailable: {exc}"


# ---------------------------------------------------------------------------
# Small CSV helpers (stdlib only; no pandas dependency).
# ---------------------------------------------------------------------------
def read_csv(path: str) -> Tuple[List[str], List[List[str]]]:
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    if not rows:
        return [], []
    return rows[0], rows[1:]


def col(header: List[str], name: str) -> Optional[int]:
    try:
        return header.index(name)
    except ValueError:
        return None


def fcol(header: List[str], data: List[List[str]], name: str) -> List[float]:
    idx = col(header, name)
    out: List[float] = []
    if idx is None:
        return out
    for r in data:
        if idx >= len(r) or r[idx] == "":
            out.append(math.nan)
            continue
        try:
            out.append(float(r[idx]))
        except ValueError:
            out.append(math.nan)
    return out


def has_data(path: str) -> bool:
    """True if the CSV exists and has at least one data row."""
    if not os.path.isfile(path):
        return False
    try:
        header, data = read_csv(path)
    except Exception:
        return False
    return bool(header) and any(any(c.strip() for c in row) for row in data)


# ---------------------------------------------------------------------------
# Individual plotters. Each returns (name, status, reason).
# ---------------------------------------------------------------------------
def _finite(vals: List[float]) -> List[float]:
    return [v for v in vals if v is not None and not math.isnan(v)]


def plot_set_vs_actual(call_dir: str, csv_path: str, out_dir: str) -> List[dict]:
    """Generic set/target-vs-actual TCP tracking, plus error, if columns exist."""
    results: List[dict] = []
    header, data = read_csv(csv_path)
    t = fcol(header, data, "t_s") or fcol(header, data, "time_s")
    if not t:
        t = list(range(len(data)))

    # target/set prefix auto-detect
    for setp in ("target", "tcp_x_set", "set"):
        pass
    axes = [("x", "target_x", "actual_x"), ("y", "target_y", "actual_y"),
            ("z", "target_z", "actual_z"),
            ("x", "tcp_x_set", "tcp_x_actual"), ("y", "tcp_y_set", "tcp_y_actual"),
            ("z", "tcp_z_set", "tcp_z_actual"),
            ("x", "set_x", "actual_x"), ("y", "set_y", "actual_y"),
            ("z", "set_z", "actual_z")]
    series = {}
    for label, s, a in axes:
        sv, av = fcol(header, data, s), fcol(header, data, a)
        if _finite(sv) or _finite(av):
            series.setdefault(label, (sv, av))

    if series:
        try:
            fig, ax = plt.subplots(figsize=(9, 5))
            for label, (sv, av) in series.items():
                if _finite(sv):
                    ax.plot(t[:len(sv)], sv, "--", label=f"{label}_set")
                if _finite(av):
                    ax.plot(t[:len(av)], av, "-", label=f"{label}_actual")
            ax.set_xlabel("t [s]")
            ax.set_ylabel("position [m]")
            ax.set_title("TCP set vs actual")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "tcp_xyz_set_vs_actual.png"), dpi=120)
            plt.close(fig)
            results.append({"plot": "tcp_xyz_set_vs_actual.png", "status": "ok"})
        except Exception as exc:
            results.append({"plot": "tcp_xyz_set_vs_actual.png", "status": "error",
                            "reason": str(exc)})

    perr = fcol(header, data, "position_error_m") or fcol(header, data, "tcp_position_error_norm")
    oerr = fcol(header, data, "orientation_error_rad") or fcol(header, data, "tcp_orientation_error_norm")
    if _finite(perr) or _finite(oerr):
        try:
            fig, ax = plt.subplots(figsize=(9, 4.5))
            if _finite(perr):
                ax.plot(t[:len(perr)], perr, label="position_error_m")
            if _finite(oerr):
                ax.plot(t[:len(oerr)], oerr, label="orientation_error_rad")
            ax.set_xlabel("t [s]")
            ax.set_title("TCP tracking error")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "tcp_error.png"), dpi=120)
            plt.close(fig)
            results.append({"plot": "tcp_error.png", "status": "ok"})
        except Exception as exc:
            results.append({"plot": "tcp_error.png", "status": "error", "reason": str(exc)})
    return results


def plot_path_xyz(csv_path: str, out_dir: str, out_name: str, title: str) -> List[dict]:
    header, data = read_csv(csv_path)
    x, y, z = fcol(header, data, "x"), fcol(header, data, "y"), fcol(header, data, "z")
    if not (_finite(x) and _finite(y)):
        return [{"plot": out_name, "status": "skipped", "reason": "no x/y columns"}]
    try:
        fig = plt.figure(figsize=(6, 6))
        try:
            ax = fig.add_subplot(111, projection="3d")
            ax.plot(x, y, z, "-o", markersize=2)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
        except Exception:
            ax = fig.add_subplot(111)
            ax.plot(x, y, "-o", markersize=2)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
        ax.set_title(title)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, out_name), dpi=120)
        plt.close(fig)
        return [{"plot": out_name, "status": "ok"}]
    except Exception as exc:
        return [{"plot": out_name, "status": "error", "reason": str(exc)}]


def plot_phase_summary(csv_path: str, out_dir: str, out_name: str) -> List[dict]:
    """Bar chart of per-phase duration from phase_summary.csv (§6 task plots)."""
    header, data = read_csv(csv_path)
    p_i = col(header, "phase")
    dur = fcol(header, data, "duration_s")
    if p_i is None or not _finite(dur):
        return [{"plot": out_name, "status": "skipped", "reason": "no phase/duration data"}]
    labels = [r[p_i] if p_i < len(r) else str(i) for i, r in enumerate(data)]
    try:
        fig, ax = plt.subplots(figsize=(max(7, len(labels) * 0.8), 4.5))
        ax.bar(range(len(dur)), [d if d == d else 0 for d in dur])
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax.set_ylabel("duration [s]")
        ax.set_title("Phase duration")
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, out_name), dpi=120)
        plt.close(fig)
        return [{"plot": out_name, "status": "ok"}]
    except Exception as exc:
        return [{"plot": out_name, "status": "error", "reason": str(exc)}]


def plot_rl_input(csv_path: str, out_dir: str, out_name: str) -> List[dict]:
    header, data = read_csv(csv_path)
    names_i = col(header, "input_name")
    raw = fcol(header, data, "raw_value")
    if names_i is None or not _finite(raw):
        return [{"plot": out_name, "status": "skipped", "reason": "no observation rows"}]
    labels = [r[names_i] if names_i < len(r) else str(i) for i, r in enumerate(data)]
    try:
        fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.5), 4.5))
        ax.bar(range(len(raw)), raw)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=60, ha="right", fontsize=7)
        ax.set_title("RL observation (raw values)")
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, out_name), dpi=120)
        plt.close(fig)
        return [{"plot": out_name, "status": "ok"}]
    except Exception as exc:
        return [{"plot": out_name, "status": "error", "reason": str(exc)}]


# ---------------------------------------------------------------------------
# Per-call driver.
# ---------------------------------------------------------------------------
def load_metadata(call_dir: str) -> Dict[str, str]:
    path = os.path.join(call_dir, "metadata.json")
    if not os.path.isfile(path):
        return {}
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return {}


def plot_call(call_dir: str, quiet: bool = False) -> bool:
    """Plot one call dir. Returns True if a plots/ dir was produced. Never raises."""
    meta = load_metadata(call_dir)
    out_dir = os.path.join(call_dir, "plots")
    status = {"call_dir": call_dir, "action": meta.get("action_name", ""),
              "log_group": meta.get("log_group", ""), "plots": [], "mpl_ok": _MPL_OK}

    try:
        os.makedirs(out_dir, exist_ok=True)
    except Exception as exc:
        # Cannot even create plots dir; give up quietly.
        if not quiet:
            print(f"[plot] cannot create {out_dir}: {exc}")
        return False

    if not _MPL_OK:
        status["skipped_reason"] = _MPL_ERR
        _write_status(out_dir, status)
        if not quiet:
            print(f"[plot] {call_dir}: {_MPL_ERR} (skipped, not an error)")
        return True

    plotted = status["plots"]

    def try_file(fname, fn, *args):
        path = os.path.join(call_dir, fname)
        if not has_data(path):
            plotted.append({"source": fname, "status": "skipped",
                            "reason": "missing or empty"})
            return
        try:
            plotted.extend({"source": fname, **r} for r in fn(path, *args))
        except Exception as exc:
            plotted.append({"source": fname, "status": "error",
                            "reason": f"{exc}\n{traceback.format_exc()}"})

    # Baseline / task TCP tracking files (various names across loggers).
    for tname in ("tcp_tracking.csv", "trajectory_tracking.csv", "tcp_tracking",
                  "move_to_pose.csv", "move_to_pose_cartesian.csv", "repeat_0001.csv"):
        p = os.path.join(call_dir, tname)
        if has_data(p):
            try:
                plotted.extend({"source": tname, **r}
                               for r in plot_set_vs_actual(call_dir, p, out_dir))
            except Exception as exc:
                plotted.append({"source": tname, "status": "error", "reason": str(exc)})

    # Task phase plots (§6).
    try_file("phase_summary.csv", plot_phase_summary, out_dir, "phase_duration.png")

    # RL planning paths.
    try_file("rl_planning_path.csv", plot_path_xyz, out_dir, "rl_path_3d.png", "RL planned path")
    try_file("planning_pick.csv", plot_path_xyz, out_dir, "rl_path_pick_3d.png", "RL pick path")
    try_file("planning_place.csv", plot_path_xyz, out_dir, "rl_path_place_3d.png", "RL place path")

    # RL observation vectors (rl_observation.csv is the current name; the
    # rl_input_15d* names are the older ones, kept for backward compatibility).
    try_file("rl_observation.csv", plot_rl_input, out_dir, "rl_observation.png")
    try_file("rl_observation_pick.csv", plot_rl_input, out_dir, "rl_observation_pick.png")
    try_file("rl_observation_place.csv", plot_rl_input, out_dir, "rl_observation_place.png")
    try_file("rl_input_15d.csv", plot_rl_input, out_dir, "rl_observation.png")
    try_file("rl_input_15d_pick.csv", plot_rl_input, out_dir, "rl_observation_pick.png")
    try_file("rl_input_15d_place.csv", plot_rl_input, out_dir, "rl_observation_place.png")

    _write_status(out_dir, status)
    if not quiet:
        ok = sum(1 for p in plotted if p.get("status") == "ok")
        print(f"[plot] {call_dir}: {ok} plot(s) written -> {out_dir}")
    return True


def _write_status(out_dir: str, status: dict) -> None:
    try:
        with open(os.path.join(out_dir, "plot_status.json"), "w") as fh:
            json.dump(status, fh, indent=2)
    except Exception:
        pass


def iter_call_dirs(root: str):
    """Yield call_* dirs under root (root itself if it is a call dir)."""
    if os.path.basename(root.rstrip("/")).startswith("call_"):
        yield root
        return
    if os.path.isfile(os.path.join(root, "metadata.json")):
        yield root
        return
    for dirpath, dirnames, _ in os.walk(root):
        for d in dirnames:
            if d.startswith("call_"):
                yield os.path.join(dirpath, d)


def main(argv: List[str]) -> int:
    args = [a for a in argv[1:] if not a.startswith("-")]
    quiet = "--quiet" in argv
    if not args:
        print(__doc__)
        return 0
    root = args[0]
    if not os.path.exists(root):
        print(f"[plot] path not found: {root}")
        return 0
    n = 0
    for call_dir in iter_call_dirs(root):
        try:
            if plot_call(call_dir, quiet=quiet):
                n += 1
        except Exception as exc:  # absolute safety net
            print(f"[plot] unexpected error on {call_dir}: {exc}")
    if not quiet:
        print(f"[plot] processed {n} call dir(s)")
    return 0


if __name__ == "__main__":
    # Best-effort tool: never propagate a non-zero exit that could fail a caller.
    try:
        sys.exit(main(sys.argv))
    except SystemExit:
        raise
    except Exception as exc:
        print(f"[plot] fatal (ignored): {exc}")
        sys.exit(0)
