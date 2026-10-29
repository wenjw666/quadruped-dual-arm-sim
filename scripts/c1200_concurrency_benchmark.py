#!/usr/bin/env python3
"""Measure concurrent SmolVLM requests and llama-server process occupancy."""
from __future__ import annotations

import argparse
import base64
import json
import os
import threading
import time
import urllib.request


def server_pid() -> int | None:
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            cmd = open(f"/proc/{name}/cmdline", "rb").read().decode("utf-8", "ignore")
        except OSError:
            continue
        if "llama-server" in cmd:
            return int(name)
    return None


def proc_sample(pid: int):
    try:
        fields = open(f"/proc/{pid}/stat").read().split()
        rss_kb = int(fields[23]) * os.sysconf("SC_PAGE_SIZE") // 1024
        ticks = (int(fields[13]) + int(fields[14])) / os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        return time.monotonic(), rss_kb, ticks
    except (OSError, ValueError):
        return time.monotonic(), 0, 0.0


def request(image_b64: str, prompt: str, url: str, out: dict, idx: int) -> None:
    payload = {"messages": [{"role": "user", "content": [
        {"type": "text", "text": prompt},
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + image_b64}},
    ]}], "temperature": 0, "max_tokens": 64}
    req = urllib.request.Request(url, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
    start = time.monotonic()
    try:
        with urllib.request.urlopen(req, timeout=180) as res:
            obj = json.load(res)
        out[idx] = {"latency_s": time.monotonic() - start, "finish_reason": obj.get("choices", [{}])[0].get("finish_reason")}
    except Exception as exc:  # noqa: BLE001
        out[idx] = {"error": repr(exc), "latency_s": time.monotonic() - start}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--concurrency", type=int, default=2)
    ap.add_argument("--url", default="http://127.0.0.1:8088/v1/chat/completions")
    args = ap.parse_args()
    encoded = base64.b64encode(open(args.image, "rb").read()).decode()
    outputs: dict[int, dict] = {}
    threads = [threading.Thread(target=request, args=(encoded, "Describe the image in one short sentence.", args.url, outputs, i)) for i in range(args.concurrency)]
    pid = server_pid()
    samples = []
    t0 = time.monotonic()
    for t in threads:
        t.start()
    while any(t.is_alive() for t in threads):
        if pid:
            samples.append(proc_sample(pid))
        time.sleep(0.2)
    for t in threads:
        t.join()
    if pid:
        samples.append(proc_sample(pid))
    peak_rss = max((s[1] for s in samples), default=0)
    if len(samples) > 1:
        elapsed = samples[-1][0] - samples[0][0]
        cpu_seconds = samples[-1][2] - samples[0][2]
        avg_cpu = 100.0 * cpu_seconds / max(elapsed * (os.cpu_count() or 1), 1e-6)
    else:
        avg_cpu = 0.0
    print(json.dumps({"pid": pid, "concurrency": args.concurrency, "wall_s": time.monotonic() - t0,
                      "peak_rss_mb": peak_rss / 1024.0, "avg_cpu_percent_of_all_cores": avg_cpu,
                      "requests": outputs}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
