#!/usr/bin/env python3
"""Run scale-30/50 ROS experiments with seed-local log paths."""
import argparse
import os
import shlex
import signal
import subprocess
import time
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--package", default="ego_planner")
    p.add_argument("--result-root", required=True)
    p.add_argument("--scales", nargs="+", type=int, default=[30, 50])
    p.add_argument("--methods", nargs="+", default=["benign", "full_attack"])
    p.add_argument("--scenario", default="random")
    p.add_argument("--seeds", type=int, default=30,
                   help="number of seeds; seeds are 1..N")
    p.add_argument("--max-time-30", type=float, default=70.0)
    p.add_argument("--max-time-50", type=float, default=120.0)
    p.add_argument("--sleep-between", type=float, default=5.0)
    p.add_argument("--timeseries-interval", type=float, default=0.2,
                   help="seconds between attacker/victim timeseries rows")
    p.add_argument("--roslaunch", default="roslaunch")
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args()


def run_one(args, scale, method, seed):
    if scale not in (30, 50):
        raise ValueError(f"unsupported scale: {scale}")
    launch = f"swarm_{scale}.launch"
    duration = args.max_time_30 if scale == 30 else args.max_time_50
    seed_dir = (Path(args.result_root).expanduser().resolve()
                / f"scale_{scale}" / args.scenario / method / f"seed_{seed}")
    seed_dir.mkdir(parents=True, exist_ok=True)

    # These are deliberately passed to roslaunch, rather than relying on XML
    # defaults. Only the attacker and victim derive fragment files here.
    cmd = [args.roslaunch, args.package, launch,
           f"experiment_scenario:={args.scenario}",
           f"attack_scenario:={args.scenario}",
           f"attack_method:={method}",
           f"experiment_seed:={seed}",
           f"experiment_swarm_size:={scale}",
           f"experiment_trial_id:={args.scenario}_{method}_seed{seed}",
           f"experiment_log_file:={seed_dir / 'trial_summary.csv'}",
           f"experiment_per_uav_log_file:={seed_dir / 'per_uav_summary.csv'}",
           f"experiment_timeseries_log_file:={seed_dir / 'timeseries.csv'}"]
    cmd.append(f"experiment_timeseries_interval:={args.timeseries_interval}")
    # shlex.join was added in Python 3.8; keep the batch runner compatible
    # with older ROS virtual-machine Python versions.
    printable_cmd = " ".join(shlex.quote(str(item)) for item in cmd)
    print("RUNNING:", printable_cmd, flush=True)
    if args.dry_run:
        return
    log_path = seed_dir / "roslaunch.log"
    with log_path.open("w", encoding="utf-8") as log:
        # Put roslaunch and all of its ROS children in one process group. On
        # timeout, SIGINT gives roslaunch time to shut down nodes and flush
        # their CSV files before a stronger signal is considered.
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                preexec_fn=os.setsid)
        try:
            proc.wait(timeout=duration)
        except subprocess.TimeoutExpired:
            print(f"TIMEOUT after {duration}s: scale={scale} method={method} seed={seed}",
                  flush=True)
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                print("Graceful shutdown timed out; sending SIGTERM", flush=True)
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()

    print("CLEANUP: rosnode kill -a", flush=True)
    try:
        cleanup = subprocess.run(
            ["rosnode", "kill", "-a"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=30,
            universal_newlines=True,
            check=False)
        if cleanup.returncode != 0:
            print("WARNING: rosnode cleanup failed:", cleanup.stdout.strip(),
                  flush=True)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print("WARNING: rosnode cleanup failed:", exc, flush=True)
    time.sleep(args.sleep_between)


def main():
    args = parse_args()
    for scale in args.scales:
        for method in args.methods:
            for seed in range(1, args.seeds + 1):
                run_one(args, scale, method, seed)


if __name__ == "__main__":
    main()
