#!/usr/bin/env python3
"""Call the board-local llama-server with one image using the OpenAI format."""
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
    parser.add_argument("--prompt", default="Describe the image briefly and list visible objects.")
    args = parser.parse_args()
    encoded = base64.b64encode(open(args.image, "rb").read()).decode("ascii")
    payload = {
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": args.prompt},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + encoded}},
        ]}],
        "temperature": 0,
        "max_tokens": 96,
    }
    request = urllib.request.Request(
        args.url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=180) as response:
        result = json.load(response)
    print(json.dumps({"latency_ms": round((time.monotonic() - started) * 1000, 1), "response": result}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
