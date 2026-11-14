#!/usr/bin/env python3
"""Summarize position-tracking CSV; these metrics do not certify hardware safety."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

def analyze(path):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    motion = [r for r in rows if r["state_name"] in {"move", "sine", "stopping", "hold"}]
    if not motion:
        raise ValueError("CSV contains no active motion samples")
    errors = [float(r["position_rad"])-float(r["reference_rad"]) for r in motion]
    intervals = [(int(b["device_ms"])-int(a["device_ms"])) & 0xffffffff
                 for a, b in zip(motion, motion[1:])]
    return dict(source_file=str(Path(path).resolve()), samples=len(rows), active_samples=len(motion),
                position_rmse_rad=math.sqrt(statistics.mean(e*e for e in errors)),
                max_abs_position_error_rad=max(abs(e) for e in errors),
                max_abs_velocity_rad_s=max(abs(float(r["velocity_rad_s"])) for r in motion),
                max_abs_torque_nm=max(abs(float(r["torque_nm"])) for r in motion),
                device_sample_interval_ms_median=statistics.median(intervals) if intervals else None,
                device_sample_interval_ms_max=max(intervals) if intervals else None,
                states=sorted({r["state_name"] for r in rows}),
                faults=sorted({r["fault_name"] for r in rows if r["fault_name"] != "ok"}))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    print(json.dumps(analyze(parser.parse_args().csv), indent=2))
