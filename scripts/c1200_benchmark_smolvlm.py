#!/usr/bin/env python3
"""Measure llama-server request latency and process RSS/CPU on C1200."""
from __future__ import annotations

import argparse
import base64
import json
import os
import threading
import time
import urllib.request


def server_pid():
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


def stat(pid):
    try:
        fields = open(f"/proc/{pid}/stat").read().split()
        rss_pages = int(fields[23])
        hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        utime = int(fields[13])
        stime = int(fields[14])
        return rss_pages * os.sysconf("SC_PAGE_SIZE") // 1024, (utime + stime) / hz
    except (OSError, ValueError):
        return 0, 0.0


def request(image_b64, prompt, url):
    payload = {"messages": [{"role": "user", "content": [
        {"type": "text", "text": prompt},
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + image_b64}},
    ]}], "temperature": 0, "max_tokens": 64}
    req = urllib.request.Request(url, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
    start = time.monotonic()
    with urllib.request.urlopen(req, timeout=180) as res:
        obj = json.load(res)
    return time.monotonic() - start, obj


def one(image_b64, prompt, url):
    pid = server_pid()
    samples = []
    done = threading.Event()
    result = {}
    def run():
        try:
            result["latency"], result["obj"] = request(image_b64, prompt, url)
        except Exception as exc:  # noqa: BLE001
            result["error"] = repr(exc)
        finally:
            done.set()
    thread = threading.Thread(target=run)
    thread.start()
    while not done.wait(0.2):
        if pid:
            rss, cpu = stat(pid)
            samples.append((time.monotonic(), rss, cpu))
    thread.join()
    if pid:
        rss, cpu = stat(pid)
        samples.append((time.monotonic(), rss, cpu))
    peak_rss = max((x[1] for x in samples), default=0)
    cpu_delta = (samples[-1][2] - samples[0][2]) if len(samples) > 1 else 0.0
    wall = (samples[-1][0] - samples[0][0]) if len(samples) > 1 else result.get("latency", 0)
    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    cpu_pct = 100.0 * cpu_delta / max(wall * os.cpu_count() * 1.0, 1e-6)
    return {"pid": pid, "latency_s": result.get("latency"), "peak_rss_mb": peak_rss / 1024.0, "avg_cpu_percent_of_8_cores": cpu_pct, "error": result.get("error")}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--url", default="http://127.0.0.1:8088/v1/chat/completions")
    args = parser.parse_args()
    image_b64 = base64.b64encode(open(args.image, "rb").read()).decode()
    results = [one(image_b64, "Describe the image in one short sentence.", args.url) for _ in range(args.runs)]
    print(json.dumps({"runs": results}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
