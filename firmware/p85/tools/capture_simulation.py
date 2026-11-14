#!/usr/bin/env python3
"""Run and archive a deterministic software-only trajectory on the Linux host."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", default=str(ROOT/"build"))
    parser.add_argument("--output", required=True, help="new output directory")
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory(prefix="p85-capture-") as temporary:
        sock = str(Path(temporary)/"server")
        with open(out/"simulator.log", "w") as daemon_log:
            daemon = subprocess.Popen([str(Path(args.build).resolve()/"p85d"), "--simulate", "--socket", sock],
                                       stdout=daemon_log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+2
                while not Path(sock).exists():
                    if time.monotonic() >= deadline or daemon.poll() is not None:
                        raise RuntimeError("simulator startup failed")
                    time.sleep(0.01)
                command = [sys.executable, str(ROOT/"host/p85ctl.py"), "--transport", "socket", "--socket", sock,
                           "--csv", str(out/"sine.csv"), "run-sine", "--kp", "5", "--kd", "0.1",
                           "--amplitude", "0.02", "--period", "1", "--cycles", "2", "--ramp", "0.5"]
                result = subprocess.run(command, capture_output=True, text=True, timeout=12)
                (out/"controller.log").write_text(result.stdout+result.stderr)
                if result.returncode:
                    raise RuntimeError(f"controller failed; see {out/'controller.log'}")
                spec = importlib.util.spec_from_file_location("analysis", ROOT/"tools/analyze_csv.py")
                analysis = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(analysis)
                summary = analysis.analyze(out/"sine.csv")
                summary["validation_scope"] = "SOFTWARE SIMULATION ONLY; ideal position plant; no CAN or R0 IO"
                summary["simulation_parameters"] = dict(kp=5, kd=0.1, amplitude_rad=0.02,
                                                         period_s=1, cycles=2, ramp_s=0.5)
                (out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
                print(json.dumps(summary, indent=2))
            finally:
                daemon.terminate()
                try:
                    daemon.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    daemon.kill()
                    daemon.wait()
if __name__ == "__main__":
    main()
