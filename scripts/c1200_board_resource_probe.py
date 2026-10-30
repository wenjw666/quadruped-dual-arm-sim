#!/usr/bin/env python3
"""Collect repeatable C1200 CPU, memory, thermal and accelerator samples."""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import subprocess
import time


def read(path: str) -> str | None:
    try:
        return pathlib.Path(path).read_text().strip()
    except OSError:
        return None


def command(*args: str) -> str:
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def process_rows(pattern: str) -> list[dict]:
    output = command("ps", "-eo", "pid=,pcpu=,pmem=,rss=,etime=,args=")
    rows = []
    for line in output.splitlines():
        if pattern not in line:
            continue
        fields = line.split(None, 5)
        if len(fields) == 6:
            rows.append({"pid": fields[0], "pcpu": fields[1], "pmem": fields[2], "rss_kb": fields[3], "etime": fields[4], "cmd": fields[5]})
    return rows


def process_ticks(pattern: str) -> dict[str, int]:
    result = {}
    for row in process_rows(pattern):
        try:
            pid = row["pid"]
            fields = pathlib.Path(f"/proc/{pid}/stat").read_text().split()
            result[pid] = int(fields[13]) + int(fields[14])
        except (OSError, IndexError, ValueError):
            pass
    return result


def sample(previous_ticks: dict[str, int] | None = None, interval: float = 1.0) -> dict:
    mem = command("free", "-b")
    load = read("/proc/loadavg")
    thermal = []
    for zone in sorted(pathlib.Path("/sys/class/thermal").glob("thermal_zone*")):
        thermal.append({"zone": zone.name, "type": read(str(zone / "type")), "temp_milli_c": read(str(zone / "temp"))})
    cpufreq = []
    for policy in sorted(pathlib.Path("/sys/devices/system/cpu/cpufreq").glob("policy*")):
        cpufreq.append({
            "policy": policy.name,
            "cur": read(str(policy / "scaling_cur_freq")),
            "min": read(str(policy / "scaling_min_freq")),
            "max": read(str(policy / "scaling_max_freq")),
            "governor": read(str(policy / "scaling_governor")),
        })
    devfreq = []
    for device in sorted(pathlib.Path("/sys/class/devfreq").glob("*")):
        item = {"device": device.name}
        for key in ("cur_freq", "min_freq", "max_freq", "governor", "name"):
            value = read(str(device / key))
            if value is not None:
                item[key] = value
        devfreq.append(item)
    devices = {}
    for pattern in ("/dev/mali0", "/dev/dri/card*", "/dev/bstn*", "/dev/bstdsp", "/dev/bst_lwnn", "/dev/video*"):
        devices[pattern] = [str(path) for path in pathlib.Path("/").glob(pattern.lstrip("/"))]
    ticks = process_ticks("llama-server")
    cpu_delta = {}
    if previous_ticks is not None:
        clk = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        for pid, value in ticks.items():
            if pid in previous_ticks:
                cpu_delta[pid] = round((value - previous_ticks[pid]) / clk / max(interval, 0.001) * 100.0, 1)
    return {
        "time": time.time(),
        "loadavg": load,
        "memory_raw": mem,
        "thermal": thermal,
        "cpufreq": cpufreq,
        "devfreq": devfreq,
        "devices": devices,
        "llama": process_rows("llama-server"),
        "llama_cpu_percent_all_cores": cpu_delta,
        "vlm": process_rows("c1200_smolvlm/lib/c1200_smolvlm/vlm_node"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    samples = []
    end = time.monotonic() + args.duration
    previous_ticks = None
    while time.monotonic() < end:
        started = time.monotonic()
        samples.append(sample(previous_ticks, args.interval))
        previous_ticks = process_ticks("llama-server")
        time.sleep(max(0.0, args.interval - (time.monotonic() - started)))
    result = {"host": command("uname", "-a"), "nproc": os.cpu_count(), "samples": samples}
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        pathlib.Path(args.output).write_text(text + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
