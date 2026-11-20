#!/usr/bin/env python3
"""End-to-end wire/CLI supervision tests against the explicit software plant."""
import csv
import importlib.util
import json
import os
import struct
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
BUILD = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT/"build"
sys.argv = [sys.argv[0], *sys.argv[2:]]
spec = importlib.util.spec_from_file_location("p85ctl", ROOT/"host/p85ctl.py")
ctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ctl)

class ClientExchangeTest(unittest.TestCase):
    def test_sigint_finishes_inflight_exchange(self):
        class InterruptedTransport:
            completed = False
            interrupt = True
            def exchange(self, request):
                if self.interrupt:
                    self.interrupt = False
                    os.kill(os.getpid(), signal.SIGINT)
                self.completed = True
                seq = struct.unpack_from("<H", request, 6)[0]
                return struct.pack("<4sBBHBBBBIiiii", b"P85R", 1, 0x80, seq,
                                   0, 4, 0, 1, 100, 0, 0, 0, 0)
        transport = InterruptedTransport()
        client = ctl.Client(transport)
        with self.assertRaises(KeyboardInterrupt):
            client.call("heartbeat")
        self.assertTrue(transport.completed)
        self.assertEqual(client.last["state"], 4)
        self.assertEqual(client.call("status")["state"], 4)


class EndToEnd(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="p85test-")
        self.path = Path(self.directory.name)
        self.sock = str(self.path/"server")
        self.daemon = None
    def tearDown(self):
        if self.daemon:
            self.daemon.terminate()
            try:
                _, errors = self.daemon.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                self.daemon.kill()
                self.daemon.communicate()
                raise
            self.assertEqual(self.daemon.returncode, 0, errors)
        self.directory.cleanup()
    def server(self, mode="--simulate", *options):
        self.daemon = subprocess.Popen([str(BUILD/"p85d"), mode, "--socket", self.sock, *options],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        for _ in range(100):
            if Path(self.sock).exists():
                return
            if self.daemon.poll() is not None:
                self.fail(str(self.daemon.communicate()))
            time.sleep(0.01)
        self.fail("daemon socket did not appear")
    def cli(self, *args, timeout=12):
        return subprocess.run([sys.executable, str(ROOT/"host/p85ctl.py"), "--transport", "socket",
                               "--socket", self.sock, *args], capture_output=True, text=True, timeout=timeout)
    def test_sine_full_cycle_and_disable(self):
        self.server()
        result = self.cli("--csv", str(self.path/"sine.csv"), "run-sine", "--kp", "5", "--kd", "0.1",
                          "--amplitude", "0.02", "--period", "1", "--cycles", "2", "--ramp", "0.5")
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('"disabled_confirmed": true', result.stdout)
        with open(self.path/"sine.csv") as stream:
            rows = list(csv.DictReader(stream))
        trajectory = [r for r in rows if r["state_name"] == "sine"]
        values = [float(r["reference_rad"]) for r in trajectory]
        self.assertGreater(len(trajectory), 200)
        self.assertGreater(max(values), 0.0198)
        self.assertLess(min(values), -0.0198)
        self.assertLessEqual(max(abs(v) for v in values), 0.02001)
        # Group a sampled peak plateau; repeated device timestamps and micro-rad
        # quantization must not create extra physical oscillation peaks.
        groups = []
        for row in trajectory:
            if float(row["reference_rad"]) > 0.0198:
                t = int(row["device_ms"])
                if not groups or t-groups[-1][-1] > 100:
                    groups.append([])
                groups[-1].append(t)
        peaks = [sum(g)/len(g) for g in groups]
        self.assertEqual(len(peaks), 2, peaks)
        self.assertAlmostEqual((peaks[1]-peaks[0])/1000, 1, delta=0.06)
        self.assertIn("stopping", {r["state_name"] for r in rows})
    def test_demo_repeats_stages_and_reports_feedback(self):
        self.server()
        result = self.cli("--csv", str(self.path/"demo.csv"), "run-demo", "--kp", "5", "--kd", "0.1",
                          "--position-amplitude", "0.01", "--move-duration", "0.2",
                          "--speed-amplitude", "0.02", "--speed-period", "0.5",
                          "--speed-cycles", "2", "--repeat", "2")
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('"disabled_confirmed": true', result.stdout)
        self.assertIn('"diagnostics": true', result.stdout)
        with open(self.path/"demo.csv") as stream:
            rows = list(csv.DictReader(stream))
        stages = {row["state_name"] for row in rows}
        self.assertTrue({"demo_point_negative", "demo_point_center_out", "demo_point_positive",
                         "demo_point_center_return", "demo_position_sine", "demo_speed_sine"} <= stages, stages)
        sine = [row for row in rows if row["state_name"] == "demo_speed_sine"]
        self.assertGreater(len(sine), 100)
        self.assertGreater(max(float(row["velocity_rad_s"]) for row in sine), 0.018)
        self.assertLess(min(float(row["velocity_rad_s"]) for row in sine), -0.018)
        self.assertTrue(all(abs(float(row["position_rad"])) < 0.05 for row in sine))
        starts = [i for i, row in enumerate(rows) if row["state_name"] == "demo_point_negative"]
        self.assertGreaterEqual(len(starts), 2)
        self.assertTrue(any(row["diag_temperature_c"] for row in rows))
        self.assertTrue(any(row["diag_feedback_fresh"] == "True" for row in rows))
    def test_move(self):
        self.server()
        result = self.cli("run-move", "--kp", "5", "--kd", "0.1", "--offset", "0.02", "--duration", "0.5")
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('"disabled_confirmed": true', result.stdout)
    def test_no_motor_never_succeeds(self):
        self.server("--no-motor")
        result = self.cli("probe")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('ack_timeout', result.stdout+result.stderr)
    def test_feedback_loss_latches_fault(self):
        self.server("--simulate", "--drop-feedback-after-ms", "1500")
        result = self.cli("run-hold", "--kp", "5", "--kd", "0.1", "--hold-seconds", "3")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('feedback_timeout', result.stdout+result.stderr)
    def test_host_lease_expires(self):
        self.server()
        transport = ctl.SocketTransport(self.sock)
        client = ctl.Client(transport)
        recorder = ctl.Recorder(None)
        try:
            client.call("hello")
            client.call("config", 5000, 100, 500000, 2000, 200000, 60)
            client.call("probe")
            ctl.wait_state(client, recorder, {2}, 1)
            client.call("enable")
            ctl.wait_state(client, recorder, {4}, 1)
            time.sleep(0.42)  # deliberately no heartbeat, no status requests
            reply = client.call("status")
            self.assertEqual((reply["state"], reply["fault"]), (9, 7))
            self.assertFalse(reply["motor_status"] & 1)
            self.assertTrue(client.call("diag")["feedback_fresh"])
        finally:
            transport.close()
    def test_sigint_stops_and_disables(self):
        self.server()
        log_path = self.path/"interrupt.log"
        with open(log_path, "w") as log:
            process = subprocess.Popen([sys.executable, str(ROOT/"host/p85ctl.py"), "--transport", "socket",
                                        "--socket", self.sock, "run-hold", "--kp", "5", "--kd", "0.1",
                                        "--hold-seconds", "10"], stdout=log, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic()+3
                while time.monotonic() < deadline:
                    if '"state_name": "hold"' in log_path.read_text():
                        break
                    time.sleep(0.01)
                else:
                    self.fail("hold state not reached")
                process.send_signal(signal.SIGINT)
                _, err = process.communicate(timeout=5)
                out = log_path.read_text()
                self.assertEqual(process.returncode, 130, out+err)
                self.assertIn('"disabled_confirmed": true', out, err)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

if __name__ == "__main__":
    unittest.main(verbosity=2)
