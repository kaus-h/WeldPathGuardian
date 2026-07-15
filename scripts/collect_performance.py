#!/usr/bin/env python3
"""Collect WeldPath Guardian latency and recovery metrics from launch runs."""

import argparse
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT_RE = re.compile(
    r"state=(?P<state>\S+) raw=(?P<raw>\d+) filtered=(?P<filtered>\d+) "
    r"rejected=(?P<rejected>\d+) plan_failures=(?P<plan_failures>\d+) "
    r"exec_faults=(?P<exec_faults>\d+) pauses=(?P<pauses>\d+) "
    r"recovery_ms=(?P<recovery>[0-9.]+) path_error=(?P<path_error>[0-9.]+) "
    r"processing_ms\(perception=(?P<perception>[0-9.]+) planning=(?P<planning>[0-9.]+)\) "
    r"latency_ms\(raw=(?P<raw_latency>[0-9.]+) filtered=(?P<filtered_latency>[0-9.]+) "
    r"plan=(?P<plan_latency>[0-9.]+)\) latest_fault=(?P<fault>\S+)"
)


SCENARIOS = [
    {
        "name": "clean",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": ["scenario:=clean", "noise_stddev:=0.0", "dropout_ratio:=0.0"],
    },
    {
        "name": "gaussian_noise_0.002",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": ["scenario:=gaussian_noise", "noise_stddev:=0.002", "dropout_ratio:=0.02"],
    },
    {
        "name": "gaussian_noise_0.006",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": ["scenario:=gaussian_noise", "noise_stddev:=0.006", "dropout_ratio:=0.06"],
    },
    {
        "name": "gaussian_noise_0.010",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": ["scenario:=gaussian_noise", "noise_stddev:=0.010", "dropout_ratio:=0.10"],
    },
    {
        "name": "missing_segment",
        "launch": "fault_demo.launch.py",
        "duration": 7.0,
        "args": ["scenario:=missing_segment"],
    },
    {
        "name": "low_confidence_recovery",
        "launch": "demo.launch.py",
        "duration": 10.0,
        "args": ["scenario:=low_confidence_recovery"],
    },
]


def percentile(values, percentile_value):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((percentile_value / 100.0) * (len(ordered) - 1))))
    return ordered[index]


def summarize(samples):
    if not samples:
        return {}

    def values(key):
        return [float(sample[key]) for sample in samples]

    raw_counts = [int(sample["raw"]) for sample in samples]
    sustained_rate = max(
        [later - earlier for earlier, later in zip(raw_counts, raw_counts[1:])] or [raw_counts[-1]]
    )

    return {
        "samples": len(samples),
        "median_perception_ms": statistics.median(values("perception")),
        "p95_perception_ms": percentile(values("perception"), 95),
        "median_planning_ms": statistics.median(values("planning")),
        "p95_planning_ms": percentile(values("planning"), 95),
        "median_end_to_end_ms": statistics.median(values("plan_latency")),
        "p95_end_to_end_ms": percentile(values("plan_latency"), 95),
        "max_sustained_input_hz": sustained_rate,
        "recovery_time_ms": max(values("recovery")),
        "path_error_m": max(values("path_error")),
        "final_state": samples[-1]["state"],
        "final_fault": samples[-1]["fault"],
        "planning_failures": int(samples[-1]["plan_failures"]),
        "execution_pauses": int(samples[-1]["pauses"]),
    }


def stop_process(process):
    if process.poll() is not None:
        return
    if os.name == "posix":
        os.killpg(process.pid, signal.SIGTERM)
    else:
        process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        if os.name == "posix":
            os.killpg(process.pid, signal.SIGKILL)
        else:
            process.kill()


def run_scenario(config, domain_id):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    launch_path = ROOT / "launch" / config["launch"]
    command = ["ros2", "launch", str(launch_path), *config["args"]]
    popen_kwargs = {
        "env": env,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.STDOUT,
        "text": True,
    }
    if os.name == "posix":
        popen_kwargs["preexec_fn"] = os.setsid

    process = subprocess.Popen(command, **popen_kwargs)
    try:
        output, _ = process.communicate(timeout=config["duration"])
    except subprocess.TimeoutExpired:
        stop_process(process)
        output, _ = process.communicate(timeout=5)
    finally:
        stop_process(process)

    lines = output.splitlines()
    samples = []
    for line in lines:
        match = REPORT_RE.search(line)
        if match:
            samples.append(match.groupdict())

    return {"name": config["name"], "samples": samples, "summary": summarize(samples), "log": lines}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="docs/performance-latest.json")
    parser.add_argument("--domain-base", type=int, default=120)
    args = parser.parse_args()

    results = []
    for offset, config in enumerate(SCENARIOS):
        print(f"running {config['name']}...", flush=True)
        result = run_scenario(config, args.domain_base + offset)
        if not result["summary"]:
            print(f"no monitor samples collected for {config['name']}", file=sys.stderr)
            return 1
        results.append(result)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}")
    print(json.dumps([{"name": r["name"], **r["summary"]} for r in results], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
