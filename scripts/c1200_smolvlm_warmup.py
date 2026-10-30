#!/usr/bin/env python3
"""Warm the board-local SmolVLM image path before ROS traffic starts.

The image is placed before the text in the OpenAI multimodal content list so
llama.cpp can reuse the expensive vision prefix when prompts change.
"""

from __future__ import annotations

import argparse
import base64
import json
import time
import urllib.request


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--url", default="http://127.0.0.1:8088/v1/chat/completions")
    parser.add_argument("--prompt", default="Warm up the vision path.")
    parser.add_argument("--max-tokens", type=int, default=1)
    args = parser.parse_args()

    encoded = base64.b64encode(open(args.image, "rb").read()).decode("ascii")
    payload = {
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + encoded}},
            {"type": "text", "text": args.prompt},
        ]}],
        "temperature": 0,
        "max_tokens": args.max_tokens,
    }
    request = urllib.request.Request(
        args.url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=180) as response:
        result = json.load(response)
    print(json.dumps({
        "warmup": True,
        "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
        "timings": result.get("timings", {}),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
