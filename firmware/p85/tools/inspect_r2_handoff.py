#!/usr/bin/env python3
"""Read only R2 SRAM to verify the CAN1 release image and an advancing heartbeat.
Two snapshots are required: image presence alone is never evidence of execution.
"""
import argparse
import hashlib
import json
import mmap
import os
from pathlib import Path
import struct
import sys
import time
from inspect_r0_image import regions_from_elf, SIZE
from inspect_can_loopback import report_address
BASE = 0x04200000
FIELDS = ("magic version core_id can1_active stage heartbeat can2_start_result can3_start_result "
          "can1_owner can2_active can3_active reserved0 reserved1 reserved2 reserved3 reserved4").split()
REQUIRED = {"p85_r2_handoff_init", "p85_r2_handoff_started", "p85_r2_handoff_tick"}

def decode(data):
    if len(data) != 64:
        raise ValueError("R2 handoff report must be 64 bytes")
    report = dict(zip(FIELDS, struct.unpack("<16I", data)))
    if report["magic"] != 0x50324331 or report["version"] != 1 or report["core_id"] != 2:
        raise ValueError("Invalid R2 handoff report identity")
    if report["stage"] not in (1,2) or any(report[k] for k in FIELDS[11:]):
        raise ValueError("Invalid R2 handoff stage/reserved words")
    if any(report[k] not in (0,1) for k in ("can1_active", "can2_active", "can3_active", "can2_start_result", "can3_start_result")):
        raise ValueError("Invalid R2 CAN configuration/status value")
    return report

def inspect(elf, first, second):
    regions = regions_from_elf(elf, base=BASE, required=REQUIRED,
                              function_prefix=("p85_r2_", "FuncCanAppTask"))
    if len(first) != SIZE or len(second) != SIZE:
        raise ValueError("R2 snapshots must both be exactly 1 MiB")
    address = report_address(elf, b"p85_r2_handoff_report", 16, base=BASE)
    offset = address - BASE
    before, after = (decode(s[offset:offset+64]) for s in (first,second))
    delta = (after["heartbeat"] - before["heartbeat"]) & 0xffffffff
    image_matches = all(s[a-BASE:a-BASE+len(data)] == data for s in (first,second) for _,a,data in regions)
    checks = {"image_matches": image_matches, "can1_disabled": before["can1_active"] == after["can1_active"] == 0,
              "can1_owner_r0": before["can1_owner"] == after["can1_owner"] == 0,
              "task_running": before["stage"] == after["stage"] == 2,
              "heartbeat_advances": 0 < delta < 0x80000000,
              "can2_can3_active": all(r[k] == 1 for r in (before,after) for k in ("can2_active", "can3_active")),
              "can2_can3_started": all(r[k] == 0 for r in (before,after) for k in ("can2_start_result", "can3_start_result"))}
    return {"status": "passed" if all(checks.values()) else "failed", "checks": checks,
            "address": hex(address), "before": before, "after": after, "heartbeat_delta": delta,
            "elf_sha256": hashlib.sha256(elf).hexdigest(), "checked_bytes_per_snapshot": sum(len(d) for _,_,d in regions)}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--second-snapshot", type=Path)
    args = parser.parse_args()
    if bool(args.snapshot) != bool(args.second_snapshot):
        parser.error("Both offline snapshots must be supplied together")
    try:
        elf = args.elf.read_bytes()
        regions_from_elf(elf, base=BASE, required=REQUIRED, function_prefix=("p85_r2_", "FuncCanAppTask"))
        report_address(elf, b"p85_r2_handoff_report", 16, base=BASE)
        if args.snapshot:
            first, second = args.snapshot.read_bytes(), args.second_snapshot.read_bytes()
            boot = None
        else:
            boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
            fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
            try:
                with mmap.mmap(fd, SIZE, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ, offset=BASE) as mem:
                    first = mem[:]
                    time.sleep(0.25)
                    second = mem[:]
            finally:
                os.close(fd)
        report = inspect(elf, first, second)
        report["boot_id"] = boot
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "passed" else 1
    except (OSError, ValueError, struct.error) as error:
        print(json.dumps({"status":"error", "error":str(error)}))
        return 2
if __name__ == "__main__":
    sys.exit(main())
