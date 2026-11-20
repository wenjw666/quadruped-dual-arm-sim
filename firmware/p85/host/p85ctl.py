#!/usr/bin/env python3
"""P85 session owner. Real MsgBox and explicit software simulation share one wire protocol."""
import argparse
import csv
import ctypes
import fcntl
import json
import math
import os
import signal
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time

STATES = ("off probing disabled enabling hold move sine stopping disabling fault "
          "demo_point_negative demo_point_center_out demo_point_positive "
          "demo_point_center_return demo_position_sine demo_speed_sine").split()
ERRORS = ("ok state value no_feedback gains_not_set motor_fault feedback_timeout host_timeout "
          "ack_timeout tracking_error overspeed temperature voltage can_tx tick_overrun sequence "
          "wire position_limit torque_limit stop_timeout queue_overflow settle_timeout").split()
OPS = dict(zip("hello heartbeat status config probe enable hold move sine stop disable clear diag demo".split(), range(14)))
ACTIVE = {3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15}

class ControlError(RuntimeError):
    pass

def name(names, index):
    return names[index] if 0 <= index < len(names) else f"unknown_{index}"

def scaled(value, scale):
    if not math.isfinite(value):
        raise ControlError("all numeric arguments must be finite")
    result = round(value * scale)
    if not -(2**31) <= result < 2**31:
        raise ControlError("argument exceeds wire range")
    return result

class SocketTransport:
    def __init__(self, path):
        self.directory = tempfile.TemporaryDirectory(prefix="p85-")
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.settimeout(0.15)
        self.sock.bind(os.path.join(self.directory.name, "client"))
        os.chmod(os.path.join(self.directory.name, "client"), 0o600)
        self.sock.connect(path)
    def exchange(self, request):
        self.sock.send(request)
        return self.sock.recv(33)
    def close(self):
        self.sock.close()
        self.directory.cleanup()

class MsgBoxTransport:
    def __init__(self, path):
        self.lib = ctypes.CDLL(str(Path(path).resolve()))
        self.lib.p85_ipc_open.restype = ctypes.c_int
        self.lib.p85_ipc_exchange.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        self.lib.p85_ipc_exchange.restype = ctypes.c_int
        self.lib.p85_ipc_close.restype = None
        result = self.lib.p85_ipc_open()
        if result != 0:
            raise ControlError(f"MsgBox open failed ({result}); check /dev/ipc_msgbox0 permissions")
    def exchange(self, request):
        incoming = ctypes.create_string_buffer(32)
        outgoing = ctypes.create_string_buffer(request, 32)
        result = self.lib.p85_ipc_exchange(outgoing, incoming, 100)
        if result != 0:
            raise ControlError(f"P85 MsgBox exchange failed ({result}); an online R core does not "
                               "prove P85 firmware is installed. No simulation fallback.")
        return incoming.raw
    def close(self):
        self.lib.p85_ipc_close()

class Client:
    def __init__(self, transport):
        self.transport = transport
        self.sequence = int.from_bytes(os.urandom(2), "little")
        self.last = None
    def call(self, operation, *arguments, check=True):
        # Finish the request/reply pair before Ctrl-C starts STOP/DISABLE.
        # Otherwise the next exchange can consume the interrupted reply and
        # fail sequence validation instead of confirming motor shutdown.
        previous = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
        try:
            return self._call(operation, *arguments, check=check)
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous)

    def _call(self, operation, *arguments, check=True):
        self.sequence = (self.sequence + 1) & 65535
        request = struct.pack("<4sBBH6i", b"P85C", 1, OPS[operation], self.sequence,
                              *(list(arguments) + [0] * (6 - len(arguments))))
        raw = self.transport.exchange(request)
        if len(raw) != 32:
            raise ControlError(f"invalid P85 reply length {len(raw)}")
        magic, version, kind, seq = struct.unpack_from("<4sBBH", raw)
        if (magic, version, seq) != (b"P85R", 1, self.sequence):
            raise ControlError("reply identity/version/sequence mismatch")
        if kind == 0x82 and operation == "diag":
            tx, rx, rejected, failed, temp, voltage, fresh, age = struct.unpack_from("<IIIIhBBI", raw, 8)
            return dict(tx_ok=tx, rx_ok=rx, rx_rejected=rejected, tx_failed=failed,
                        temperature_c=temp, voltage_v=voltage, feedback_fresh=bool(fresh), feedback_age_ms=age)
        if kind != 0x80:
            raise ControlError(f"unexpected reply kind {kind}")
        result, state, fault, status, now, p, v, torque, ref = struct.unpack_from("<BBBBIiiii", raw, 8)
        reply = dict(result=result, state=state, fault=fault, motor_status=status,
                     device_ms=now, position_rad=p/1e6, velocity_rad_s=v/1e6,
                     torque_nm=torque/1e6, reference_rad=ref/1e6,
                     state_name=name(STATES, state), fault_name=name(ERRORS, fault))
        self.last = reply
        if check and result:
            raise ControlError(f"{operation}: {name(ERRORS, result)}; state={reply['state_name']}, "
                               f"fault={reply['fault_name']}")
        return reply

class Recorder:
    def __init__(self, path):
        self.file = open(path, "x", newline="") if path else None
        self.writer = None
        self.previous = None
        self.latest_diagnostic = dict(diag_tx_ok=None, diag_rx_ok=None, diag_rx_rejected=None,
                                      diag_tx_failed=None, diag_temperature_c=None, diag_voltage_v=None,
                                      diag_feedback_fresh=None, diag_feedback_age_ms=None)
    def record(self, reply):
        row = dict(host_monotonic_s=round(time.monotonic(), 6), **reply, **self.latest_diagnostic)
        if self.file:
            if self.writer is None:
                self.writer = csv.DictWriter(self.file, fieldnames=list(row))
                self.writer.writeheader()
            self.writer.writerow(row)
        transition = (reply["state"], reply["fault"])
        now = time.monotonic()
        if transition != self.previous or now-getattr(self, "last_print", 0.0) >= 0.1:
            print(json.dumps(reply), flush=True)
            self.previous = transition
            self.last_print = now
    def diagnostic(self, diag):
        self.latest_diagnostic = dict(
            diag_tx_ok=diag["tx_ok"], diag_rx_ok=diag["rx_ok"],
            diag_rx_rejected=diag["rx_rejected"], diag_tx_failed=diag["tx_failed"],
            diag_temperature_c=diag["temperature_c"], diag_voltage_v=diag["voltage_v"],
            diag_feedback_fresh=diag["feedback_fresh"], diag_feedback_age_ms=diag["feedback_age_ms"])
        print(json.dumps(dict(diagnostics=True, **diag)), flush=True)
    def close(self):
        if self.file:
            self.file.close()

def wait_state(client, recorder, desired, timeout, allow_fault=False, minimum=0):
    deadline, earliest, next_sample = time.monotonic()+timeout, time.monotonic()+minimum, time.monotonic()
    while True:
        reply = client.call("status")  # status is also a lease heartbeat
        recorder.record(reply)
        now = time.monotonic()
        if reply["state"] == 9 and not allow_fault:
            raise ControlError(f"latched fault: {reply['fault_name']}")
        if reply["state"] in desired and now >= earliest:
            return reply
        if now >= deadline:
            raise ControlError(f"state timeout waiting for {sorted(desired)}; last={reply['state_name']}")
        next_sample += 0.01
        time.sleep(max(0, next_sample-time.monotonic()))
        next_sample = max(next_sample, time.monotonic()-0.01)

def safe_disable(client, recorder, graceful):
    """A fresh disabled feedback is required; a sent command alone is insufficient."""
    try:
        reply = client.call("status")
        if graceful and reply["state"] in ACTIVE:
            client.call("stop")
            wait_state(client, recorder, {4}, 4)
    except (ControlError, OSError):
        pass
    client.call("disable")
    reply = wait_state(client, recorder, {2, 9}, 1, allow_fault=True)
    deadline = time.monotonic()+0.8
    while True:
        diag = client.call("diag")
        reply = client.call("status")
        if diag["feedback_fresh"] and not (reply["motor_status"] & 1):
            print(json.dumps(dict(disabled_confirmed=True, **diag)), flush=True)
            return
        if time.monotonic() >= deadline:
            raise ControlError("disable NOT confirmed by fresh motor feedback")
        time.sleep(0.01)

def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--transport", choices=["ipc", "socket"], default="ipc")
    p.add_argument("--socket", help="explicit simulator socket, never a hardware CAN interface")
    p.add_argument("--library", default=str(Path(__file__).resolve().parents[1]/"build/libp85_msgbox.so"))
    p.add_argument("--csv", help="new CSV file; existing files are never overwritten")
    sub = p.add_subparsers(dest="command", required=True)
    for command in ["status", "probe", "disable", "clear"]:
        sub.add_parser(command)
    for command in ["run-sine", "run-move", "run-hold", "run-demo"]:
        q = sub.add_parser(command)
        q.add_argument("--kp", type=float, required=True)
        q.add_argument("--kd", type=float, required=True)
        q.add_argument("--max-speed", type=float, default=0.5)
        q.add_argument("--max-torque", type=float, default=2)
        q.add_argument("--excursion", type=float, default=0.2)
        q.add_argument("--temperature", type=int, default=60)
        q.add_argument("--hold-seconds", type=float, default=0.5)
        if command == "run-sine":
            q.add_argument("--amplitude", type=float, required=True)
            q.add_argument("--period", type=float, required=True)
            q.add_argument("--cycles", type=int, required=True)
            q.add_argument("--ramp", type=float, default=1)
        if command == "run-move":
            q.add_argument("--offset", type=float, required=True)
            q.add_argument("--duration", type=float, required=True)
        if command == "run-demo":
            q.add_argument("--position-amplitude", type=float, default=0.01)
            q.add_argument("--move-duration", type=float, default=2.0)
            q.add_argument("--speed-amplitude", type=float, default=0.03)
            q.add_argument("--speed-period", type=float, default=4.0)
            q.add_argument("--speed-cycles", type=int, default=1)
            q.add_argument("--repeat", type=int, default=1)
    return p

def main(argv=None):
    args = parser().parse_args(argv)
    if args.transport == "socket" and not args.socket:
        print("P85 ERROR: --transport socket requires --socket", file=sys.stderr)
        return 1
    # flock covers the entire owner session; all official host invocations share it.
    lock_path = "/run/lock/p85_control.lock" if args.transport == "ipc" else f"{args.socket}.owner.lock"
    transport = recorder = None
    try:
        with open(lock_path, "a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise ControlError("another P85 controller owns this transport") from error
            if args.transport == "socket" and not args.socket:
                raise ControlError("--transport socket requires --socket")
            transport = MsgBoxTransport(args.library) if args.transport == "ipc" else SocketTransport(args.socket)
            client = Client(transport)
            recorder = Recorder(args.csv)
            recorder.record(client.call("hello"))
            if args.command == "status":
                print(json.dumps(client.call("diag")), flush=True)
            elif args.command == "probe":
                client.call("probe")
                wait_state(client, recorder, {2}, 1)
                print(json.dumps(client.call("diag")), flush=True)
            elif args.command == "clear":
                recorder.record(client.call("clear"))
            elif args.command == "disable":
                safe_disable(client, recorder, False)
            else:
                if not math.isfinite(args.hold_seconds) or not 0 <= args.hold_seconds <= 60:
                    raise ControlError("hold-seconds must be in [0, 60]")
                try:
                    client.call("config", scaled(args.kp, 1000), scaled(args.kd, 1000),
                                scaled(args.max_speed, 1e6), scaled(args.max_torque, 1000),
                                scaled(args.excursion, 1e6), args.temperature)
                    client.call("probe")
                    wait_state(client, recorder, {2}, 1)
                    client.call("enable")
                    wait_state(client, recorder, {4}, 1)
                    if args.command == "run-sine":
                        client.call("sine", scaled(args.amplitude, 1e6), scaled(args.period, 1000),
                                    args.cycles, scaled(args.ramp, 1000))
                        wait_state(client, recorder, {4}, args.period*args.cycles+2*args.ramp+5)
                    elif args.command == "run-move":
                        client.call("move", scaled(args.offset, 1e6), scaled(args.duration, 1000))
                        wait_state(client, recorder, {4}, args.duration+5)
                    elif args.command == "run-demo":
                        if not 1 <= args.speed_cycles <= 100 or not 1 <= args.repeat <= 100:
                            raise ControlError("speed-cycles and repeat must be in [1, 100]")
                        client.call("demo", scaled(args.position_amplitude, 1e6),
                                    scaled(args.move_duration, 1000),
                                    scaled(args.speed_amplitude, 1e6),
                                    scaled(args.speed_period, 1000),
                                    args.speed_cycles, args.repeat)
                        timeout = args.repeat*(4*(args.move_duration+3.5)+
                                               3*args.move_duration+3.5+
                                               args.speed_period*args.speed_cycles+3.5)+5
                        next_diag = time.monotonic()
                        deadline = time.monotonic()+timeout
                        while True:
                            reply = client.call("status")
                            recorder.record(reply)
                            now = time.monotonic()
                            if reply["state"] == 9:
                                raise ControlError(f"latched fault: {reply['fault_name']}")
                            if reply["state"] == 4:
                                break
                            if now >= deadline:
                                raise ControlError(f"demo timeout; last={reply['state_name']}")
                            if now >= next_diag:
                                recorder.diagnostic(client.call("diag"))
                                next_diag = time.monotonic()+1.0
                            time.sleep(0.01)
                    wait_state(client, recorder, {4}, args.hold_seconds+1, minimum=args.hold_seconds)
                finally:
                    original_error = sys.exc_info()[1]
                    try:
                        safe_disable(client, recorder, True)
                    except (ControlError, OSError) as shutdown_error:
                        if original_error is None:
                            raise
                        print(f"P85 shutdown: {shutdown_error}", file=sys.stderr)
            return 0
    except KeyboardInterrupt:
        print("interrupted; control session ended", file=sys.stderr)
        return 130
    except (ControlError, OSError, ValueError, struct.error) as error:
        print(f"P85 ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if recorder:
            recorder.close()
        if transport:
            transport.close()

if __name__ == "__main__":
    sys.exit(main())
