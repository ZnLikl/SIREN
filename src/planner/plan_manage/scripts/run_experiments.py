#!/usr/bin/env python3
"""Batch runner for MIRAGE / phantom trajectory poisoning experiments.

Outputs per run:
- trial_summary.csv: owner/victim-level summary row.
- per_uav_summary.csv: merged per-UAV rows from all drones.
- timeseries.csv.drone_<id>.csv: per-UAV time series for plotting.

Outputs in out-dir:
- summary.csv: concatenated trial summaries.
- per_uav.csv: concatenated per-UAV summaries, required for ripple-effect analysis.
- timeseries.csv: concatenated time-series rows for all drones and runs.
"""
import argparse
import csv
import os
import shlex
import signal
import subprocess
import time
from pathlib import Path


def run_one(base_cmd, args, timeout, log_path):
    cmd = base_cmd + " " + " ".join(f"{k}:={v}" for k, v in args.items())
    print("[RUN]", cmd)
    with open(log_path, "w") as f:
        proc = subprocess.Popen(shlex.split(cmd), stdout=f, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print("[TIMEOUT] stopping", cmd)
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        time.sleep(5)
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    return proc.returncode


def append_last_row(src_csv: Path, dst_csv: Path):
    if not src_csv.exists() or src_csv.stat().st_size == 0:
        return False
    with open(src_csv, newline="") as src:
        rows = list(csv.reader(src))
    if len(rows) < 2:
        return False
    header, row = rows[0], rows[-1]
    write_header = not dst_csv.exists() or dst_csv.stat().st_size == 0
    with open(dst_csv, "a", newline="") as dst:
        writer = csv.writer(dst)
        if write_header:
            writer.writerow(header)
        writer.writerow(row)
    return True


def merge_per_uav_fragments(prefix: Path, merged_csv: Path):
    """Merge files named '<prefix>.drone_<id>.csv' into merged_csv."""
    fragments = sorted(prefix.parent.glob(prefix.name + ".drone_*.csv"))
    if not fragments:
        return 0
    if merged_csv.exists():
        merged_csv.unlink()
    count = 0
    for frag in fragments:
        if append_last_row(frag, merged_csv):
            count += 1
    return count


def append_all_rows(src_csv: Path, dst_csv: Path):
    if not src_csv.exists() or src_csv.stat().st_size == 0:
        return 0
    with open(src_csv, newline="") as src:
        rows = list(csv.reader(src))
    if len(rows) < 2:
        return 0
    write_header = not dst_csv.exists() or dst_csv.stat().st_size == 0
    with open(dst_csv, "a", newline="") as dst:
        writer = csv.writer(dst)
        if write_header:
            writer.writerow(rows[0])
        for row in rows[1:]:
            writer.writerow(row)
    return len(rows) - 1


def merge_timeseries_fragments(prefix: Path, merged_csv: Path):
    """Merge files named '<prefix>.drone_<id>.csv' into merged_csv, preserving all rows."""
    fragments = sorted(prefix.parent.glob(prefix.name + ".drone_*.csv"))
    if not fragments:
        return 0
    if merged_csv.exists():
        merged_csv.unlink()
    count = 0
    for frag in fragments:
        count += append_all_rows(frag, merged_csv)
    return count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--launch", required=True, help="Base roslaunch command, e.g. 'roslaunch ego_planner swarm.launch'")
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--seeds", nargs="+", type=int, default=list(range(10)))
    ap.add_argument("--scenarios", nargs="+", default=["herding", "trap", "goal_oscillation"])
    ap.add_argument("--methods", nargs="+", default=["benign", "random_fake", "no_temporal_refresh", "no_impact_objective", "full_attack", "naive_infeasible"])
    ap.add_argument("--swarm-size", type=int, default=10)
    ap.add_argument("--attacker-count", type=int, default=1)
    ap.add_argument("--victim-id", type=int, default=9)
    ap.add_argument("--malicious-id", type=int, default=0)
    ap.add_argument("--map", default="random_forest")
    ap.add_argument("--comm-radius", type=float, default=-1.0)
    ap.add_argument("--sharing-freq", type=float, default=-1.0)
    ap.add_argument("--log-owner-drone-id", type=int, default=-9999,
                    help="Default -9999 lets the C++ logger use victim-id as trial-summary owner.")
    ns = ap.parse_args()

    out_dir = Path(ns.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = out_dir / "summary.csv"
    per_uav_all_csv = out_dir / "per_uav.csv"
    timeseries_all_csv = out_dir / "timeseries.csv"
    for p in [summary_csv, per_uav_all_csv, timeseries_all_csv]:
        if p.exists():
            p.unlink()

    missing = []
    missing_per_uav = []
    missing_timeseries = []
    for scenario in ns.scenarios:
        for method in ns.methods:
            for seed in ns.seeds:
                run_dir = out_dir / scenario / method / f"seed_{seed}"
                run_dir.mkdir(parents=True, exist_ok=True)
                log_file = run_dir / "roslaunch.log"
                trial_csv = run_dir / "trial_summary.csv"
                per_uav_prefix = run_dir / "per_uav_summary.csv"
                per_uav_merged = run_dir / "per_uav_summary_merged.csv"
                timeseries_prefix = run_dir / "timeseries.csv"
                timeseries_merged = run_dir / "timeseries_merged.csv"
                trial_id = f"{ns.map}_{scenario}_{method}_seed{seed}"

                # Remove stale fragments before launching.
                for stale in run_dir.glob("per_uav_summary.csv.drone_*.csv"):
                    stale.unlink()
                for stale in run_dir.glob("timeseries.csv.drone_*.csv"):
                    stale.unlink()

                args = {
                    "attack_enable": "false" if method == "benign" else "true",
                    "attack_method": method,
                    "attack_scenario": scenario,
                    "attack_malicious_drone_id": ns.malicious_id,
                    "attack_victim_drone_id": ns.victim_id,
                    "experiment_scenario": scenario,
                    "experiment_seed": seed,
                    "experiment_swarm_size": ns.swarm_size,
                    "experiment_attacker_count": ns.attacker_count,
                    "experiment_trial_id": trial_id,
                    "experiment_map": ns.map,
                    "experiment_comm_radius": ns.comm_radius,
                    "experiment_sharing_freq": ns.sharing_freq,
                    "experiment_log_owner_drone_id": ns.log_owner_drone_id,
                    "experiment_log_file": str(trial_csv),
                    "experiment_per_uav_log_file": str(per_uav_prefix),
                    "experiment_per_uav_log_enable": "true",
                    "experiment_timeseries_log_file": str(timeseries_prefix),
                    "experiment_timeseries_log_enable": "true",
                    "experiment_timeseries_interval": "0.2",
                }
                run_one(ns.launch, args, ns.timeout, log_file)
                if not append_last_row(trial_csv, summary_csv):
                    missing.append(str(run_dir))

                n_per = merge_per_uav_fragments(per_uav_prefix, per_uav_merged)
                if n_per <= 0:
                    missing_per_uav.append(str(run_dir))
                else:
                    with open(per_uav_merged, newline="") as src:
                        rows = list(csv.reader(src))
                    write_header = not per_uav_all_csv.exists() or per_uav_all_csv.stat().st_size == 0
                    with open(per_uav_all_csv, "a", newline="") as dst:
                        writer = csv.writer(dst)
                        if write_header:
                            writer.writerow(rows[0])
                        for row in rows[1:]:
                            writer.writerow(row)

                n_ts = merge_timeseries_fragments(timeseries_prefix, timeseries_merged)
                if n_ts <= 0:
                    missing_timeseries.append(str(run_dir))
                else:
                    append_all_rows(timeseries_merged, timeseries_all_csv)

    print("[DONE] summary:", summary_csv)
    print("[DONE] per-UAV:", per_uav_all_csv)
    print("[DONE] timeseries:", timeseries_all_csv)
    if missing:
        print("[WARN] missing or empty trial_summary.csv in:")
        for m in missing:
            print("  ", m)
    if missing_per_uav:
        print("[WARN] missing per-UAV fragments in:")
        for m in missing_per_uav:
            print("  ", m)
    if missing_timeseries:
        print("[WARN] missing timeseries fragments in:")
        for m in missing_timeseries:
            print("  ", m)


if __name__ == "__main__":
    main()
