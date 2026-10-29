#!/usr/bin/env python3
"""Measure sequential board-local SmolVLM requests and process memory."""
from __future__ import annotations

import argparse
import base64
import json
import os
import subprocess
import time
import urllib.request


def server_pid() -> int | None:
    try:
        out = subprocess.check_output(["pidof", "llama-server"], text=True).strip().split()
        return int(out[0]) if out else None
    except (OSError, subprocess.CalledProcessError, ValueError):
        return None


def process_sample(pid: int | None) -> dict:
    if pid is None:
        return {}
    try:
        values = {}
        for line in open(f"/proc/{pid}/status", encoding="utf-8"):
            key, _, value = line.partition(":")
            if key in {"VmRSS", "VmSize", "Threads"}:
                values[key] = value.strip()
        stat = subprocess.check_output(
            ["ps", "-p", str(pid), "-o", "pcpu=,pmem=,etime="], text=True
        ).strip()
        values["ps"] = stat
        return values
    except (OSError, subprocess.CalledProcessError):
        return {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--url", default="http://127.0.0.1:8088/v1/chat/completions")
    args = parser.parse_args()
    image = base64.b64encode(open(args.image, "rb").read()).decode("ascii")
    payload = {
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": "Describe the image briefly."},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + image}},
        ]}],
        "temperature": 0,
        "max_tokens": 64,
    }
    body = json.dumps(payload).encode("utf-8")
    samples = []
    pid = server_pid()
    for index in range(args.count):
        started = time.monotonic()
        request = urllib.request.Request(
            args.url, data=body, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(request, timeout=180) as response:
            result = json.load(response)
        samples.append({
            "index": index,
            "latency_ms": round((time.monotonic() - started) * 1000.0, 1),
            "completion_tokens": result.get("usage", {}).get("completion_tokens"),
            "timings": result.get("timings", {}),
            "process": process_sample(pid),
        })
    latencies = [item["latency_ms"] for item in samples]
    print(json.dumps({
        "pid": pid,
        "count": args.count,
        "mean_latency_ms": round(sum(latencies) / len(latencies), 1) if latencies else None,
        "min_latency_ms": min(latencies) if latencies else None,
        "max_latency_ms": max(latencies) if latencies else None,
        "samples": samples,
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
