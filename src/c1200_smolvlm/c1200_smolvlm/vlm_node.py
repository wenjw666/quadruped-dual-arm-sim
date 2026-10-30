"""ROS2 bridge around the C1200 ARM64 llama.cpp multimodal CLI.

The first board milestone intentionally invokes the model per request.  This
keeps the model boundary observable and lets us validate the exact GGUF/mmproj
pair before introducing a persistent server or a NPU/Mali backend.  The node
publishes text/JSON only; an independent application must validate any robot
command before it reaches an actuator.
"""

from __future__ import annotations

import json
import base64
import os
import subprocess
import tempfile
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VLMNode(Node):
    def __init__(self) -> None:
        super().__init__("c1200_smolvlm")
        self.image_topic = str(self.declare_parameter("image_topic", "/smolvlm/image").value)
        self.prompt_topic = str(self.declare_parameter("prompt_topic", "/smolvlm/prompt").value)
        self.response_topic = str(self.declare_parameter("response_topic", "/smolvlm/response").value)
        self.status_topic = str(self.declare_parameter("status_topic", "/smolvlm/status").value)
        self.runner = str(self.declare_parameter("runner", "/userdata/smolvlm/build/bin/llama-mtmd-cli").value)
        self.backend = str(self.declare_parameter("backend", "server").value).lower()
        self.server_url = str(self.declare_parameter("server_url", "http://127.0.0.1:8088/v1/chat/completions").value)
        self.model = str(self.declare_parameter("model", "/userdata/smolvlm/models/SmolVLM-500M-Instruct-Q8_0.gguf").value)
        self.mmproj = str(self.declare_parameter("mmproj", "/userdata/smolvlm/models/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf").value)
        self.threads = int(self.declare_parameter("threads", 8).value)
        # ``Node.context`` is an rclpy property, so keep the CLI setting under
        # a distinct name.
        self.ctx_size = int(self.declare_parameter("context", 2048).value)
        self.max_tokens = int(self.declare_parameter("max_tokens", 128).value)
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 120.0).value)
        self.offload_mmproj = bool(self.declare_parameter("offload_mmproj", False).value)
        self.image_quality = int(self.declare_parameter("image_quality", 90).value)
        self._image = None
        self._image_seq = 0
        self._image_lock = threading.Lock()
        self._busy = False
        self._busy_lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="smolvlm")
        self._image_sub = self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self._prompt_sub = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)
        self._response_pub = self.create_publisher(String, self.response_topic, 10)
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._heartbeat_timer = self.create_timer(2.0, self._heartbeat)
        self._publish_status("ready")

    def _heartbeat(self) -> None:
        with self._image_lock:
            sequence = self._image_seq
        self._publish_status(f"heartbeat image_seq={sequence}")

    def _on_image(self, msg: Image) -> None:
        try:
            import cv2
            import numpy as np

            if msg.encoding in ("bgr8", "rgb8"):
                channels = 3
            elif msg.encoding in ("mono8", "8UC1"):
                channels = 1
            else:
                self._publish_status(f"unsupported_encoding:{msg.encoding}")
                return
            array = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            expected = int(msg.height) * int(msg.step)
            if array.size < expected or msg.height <= 0 or msg.width <= 0:
                return
            array = array[:expected].reshape((int(msg.height), int(msg.step)))
            array = array[:, : int(msg.width) * channels].reshape((int(msg.height), int(msg.width), channels))
            if msg.encoding == "rgb8":
                array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            if channels == 1:
                array = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
            with self._image_lock:
                self._image = array.copy()
                self._image_seq += 1
                sequence = self._image_seq
            if sequence == 1 or sequence % 20 == 0:
                self._publish_status(f"image_received seq={sequence} size={msg.width}x{msg.height}")
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"image_error:{exc}")

    def _on_prompt(self, msg: String) -> None:
        prompt = msg.data.strip()
        if not prompt:
            self._publish_status("empty_prompt")
            return
        with self._busy_lock:
            if self._busy:
                self._publish_status("busy_drop_prompt")
                return
            self._busy = True
        with self._image_lock:
            image = None if self._image is None else self._image.copy()
            image_seq = self._image_seq
        if image is None:
            self._response_pub.publish(String(data=json.dumps({"ok": False, "error": "no_image"})))
            self._publish_status("no_image")
            with self._busy_lock:
                self._busy = False
            return
        self._publish_status(f"queued image_seq={image_seq}")
        self._executor.submit(self._infer, prompt, image, image_seq)

    def _infer(self, prompt, image, image_seq: int) -> None:
        import cv2

        started = time.monotonic()
        temp_path: Optional[str] = None
        try:
            if self.backend == "server":
                ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality])
                if not ok:
                    raise RuntimeError("failed to encode image")
                payload = {
                    "messages": [{"role": "user", "content": [
                        {"type": "image_url", "image_url": {
                            "url": "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
                        }},
                        {"type": "text", "text": prompt},
                    ]}],
                    "temperature": 0,
                    "max_tokens": self.max_tokens,
                }
                request = urllib.request.Request(
                    self.server_url,
                    data=json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                )
                with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                    result = json.load(response)
                text = result["choices"][0]["message"]["content"]
                payload = {
                    "ok": True,
                    "text": text,
                    "image_seq": image_seq,
                    "latency_ms": round((time.monotonic() - started) * 1000.0, 1),
                    "backend": "llama-server",
                }
                self._response_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
                self._publish_status(f"done image_seq={image_seq} latency_ms={payload['latency_ms']}")
                return
            if self.backend != "cli":
                raise RuntimeError(f"unsupported backend: {self.backend}")
            with tempfile.NamedTemporaryFile(prefix="smolvlm-", suffix=".jpg", delete=False) as handle:
                temp_path = handle.name
            if not cv2.imwrite(temp_path, image, [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]):
                raise RuntimeError("failed to encode image")
            if not (Path(self.runner).is_file() and os.access(self.runner, os.X_OK)):
                raise RuntimeError(f"runner unavailable: {self.runner}")
            if not Path(self.model).is_file():
                raise RuntimeError(f"model unavailable: {self.model}")
            if not Path(self.mmproj).is_file():
                raise RuntimeError(f"mmproj unavailable: {self.mmproj}")
            command = [
                self.runner,
                "-m", self.model,
                "--mmproj", self.mmproj,
                "--image", temp_path,
                "-t", str(self.threads),
                "-c", str(self.ctx_size),
                "-n", str(self.max_tokens),
                "-p", prompt,
            ]
            if not self.offload_mmproj:
                command.append("--no-mmproj-offload")
            env = os.environ.copy()
            runner_dir = str(Path(self.runner).resolve().parent)
            env["LD_LIBRARY_PATH"] = runner_dir + ":" + env.get("LD_LIBRARY_PATH", "")
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=self.timeout_sec, env=env
            )
            text = result.stdout.strip()
            if result.returncode != 0:
                raise RuntimeError(f"runner_exit={result.returncode}: {result.stderr[-1000:]}")
            payload = {
                "ok": True,
                "text": text,
                "image_seq": image_seq,
                "latency_ms": round((time.monotonic() - started) * 1000.0, 1),
                "runner": self.runner,
            }
            self._response_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            self._publish_status(f"done image_seq={image_seq} latency_ms={payload['latency_ms']}")
        except Exception as exc:  # noqa: BLE001
            payload = {
                "ok": False,
                "error": str(exc),
                "image_seq": image_seq,
                "latency_ms": round((time.monotonic() - started) * 1000.0, 1),
            }
            self._response_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            self._publish_status(f"error:{exc}")
        finally:
            if temp_path:
                try:
                    os.unlink(temp_path)
                except OSError:
                    pass
            with self._busy_lock:
                self._busy = False

    def _publish_status(self, status: str) -> None:
        self._status_pub.publish(String(data=status))

    def destroy_node(self):  # type: ignore[override]
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
