#!/usr/bin/env python3
"""Read the C1200 Safety AON startup record; this does not prove R0 execution.

The layout is from public sf-app C1200SW_V1.6_RC0930 aon_sram_ops.h.
Only the fixed 32 KiB SRAM window is mapped, read-only. No reset, Flash, CAN,
or register writes are available. Use --snapshot for offline inspection.
"""
import argparse
import json
import mmap
import os
from pathlib import Path
import struct
import sys

BASE = 0x06000000
SIZE = 32768
HEADER_OFFSET = 30720
MAGIC = 0xCAFEFEED
RECORD = struct.Struct("<H16sHQIBB12sH")
RECORD_COUNT = 23
STATES = {0: "default", 1: "poweroff", 2: "suspend", 3: "starting",
          4: "normal", 65535: "error"}
REASONS = {1: "cold", 2: "str_suspend", 3: "str_poweroff", 4: "reboot",
           5: "panic", 6: "watchdog", 7: "other"}


def inspect(data):
    if len(data) != SIZE:
        raise ValueError("AON snapshot must be exactly 32 KiB")
    magic, version = struct.unpack_from("<II", data, HEADER_OFFSET)
    if magic != MAGIC or version != 0:
        raise ValueError("Unsupported AON magic/version; do not infer startup state")
    records = []
    names = set()
    for index in range(RECORD_COUNT):
        offset = HEADER_OFFSET + 8 + index * RECORD.size
        raw = data[offset:offset + RECORD.size]
        if raw == bytes(RECORD.size):
            continue
        start, name, status, _, _, reason, count, _, end = RECORD.unpack(raw)
        if start != 1 or end != 2:
            raise ValueError("Invalid AON record markers at index " + str(index))
        nul = name.find(b"\0")
        if nul < 1 or any(c < 32 or c > 126 for c in name[:nul]):
            raise ValueError("Invalid AON name at index " + str(index))
        name = name[:nul].decode("ascii")
        if name in names:
            raise ValueError("Duplicate AON domain: " + name)
        names.add(name)
        records.append({"index": index, "domain": name, "state": status,
                        "state_name": STATES.get(status, "unknown"),
                        "start_reason": reason,
                        "start_reason_name": REASONS.get(reason, "unknown"),
                        "start_count": count})
    rt0 = next((r for r in records if r["domain"] == "realtime0"), None)
    if rt0 is None:
        raise ValueError("AON record has no realtime0 domain")
    return {"layout": "C1200SW_V1.6_RC0930/version0", "aon_base": hex(BASE),
            "meaning": "Safety startup bookkeeping; R0 execution requires matching image, P85 HELLO and advancing diagnostics",
            "rt0": rt0, "records": records}


def read_board():
    fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, SIZE, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ,
                       offset=BASE) as mapped:
            return mapped[:]
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, help="Offline full 32 KiB AON SRAM dump")
    args = parser.parse_args()
    try:
        report = inspect(args.snapshot.read_bytes() if args.snapshot else read_board())
        report["source"] = "snapshot" if args.snapshot else "board_read_only"
        if not args.snapshot:
            report["boot_id"] = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
            report["uptime_seconds"] = float(Path("/proc/uptime").read_text().split()[0])
        print(json.dumps(report, indent=2))
        return 0
    except (OSError, ValueError, struct.error) as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
