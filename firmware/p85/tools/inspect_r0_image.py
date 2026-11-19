#!/usr/bin/env python3
"""Read C1200 RT0 SRAM and compare P85 code/rodata with a build ELF.

This proves image presence only. HELLO and advancing OFF diagnostics are still
required to prove execution. No Flash, MMIO, reset or motor writes are performed.
Use --snapshot for offline analysis of a previously captured 1 MiB SRAM image.
"""
import argparse
import hashlib
import json
import mmap
import os
from pathlib import Path
import struct
import sys

BASE = 0x04000000
SIZE = 0x100000
REQUIRED = {"p85_board_create_tasks", "p85_runtime_step", "p85_hanhai_receive_once"}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def regions_from_elf(data, *, base=BASE, required=REQUIRED, function_prefix="p85_"):
    def unpack(fmt, offset):
        if offset < 0 or offset + struct.calcsize(fmt) > len(data):
            raise ValueError("Truncated ELF structure")
        return struct.unpack_from(fmt, data, offset)

    def block(offset, size):
        if offset < 0 or size < 0 or offset + size > len(data):
            raise ValueError("Truncated ELF section")
        return data[offset:offset + size]

    def name_at(strings, offset):
        end = strings.find(b"\0", offset)
        if offset < 0 or offset >= len(strings) or end < 0:
            raise ValueError("Invalid ELF string offset")
        return strings[offset:end].decode("ascii")

    header = unpack("<16sHHIIIIIHHHHHH", 0)
    if header[0][:7] != b"\x7fELF\x01\x01\x01" or header[1:3] != (2, 40):
        raise ValueError("Expected little-endian ELF32 ARM executable")
    if header[4] != base or header[11] != 40 or not header[12]:
        raise ValueError("Unexpected RT0 entry point or section table")
    sections = [unpack("<10I", header[6] + i * 40) for i in range(header[12])]
    if header[13] >= len(sections):
        raise ValueError("Invalid section name table")
    names = block(sections[header[13]][4], sections[header[13]][5])
    functions = {}
    rodata = []
    for section in sections:
        name = name_at(names, section[0])
        if name in (".rodata", ".rodata1"):
            if section[1] != 1 or section[2] & 3 != 2 or not section[5]:
                raise ValueError("Expected immutable allocated rodata")
            rodata.append((name, section[3], block(section[4], section[5])))
        if section[1] != 2:
            continue
        if section[6] >= len(sections) or section[9] != 16 or section[5] % 16:
            raise ValueError("Invalid ELF symbol table")
        strings_section = sections[section[6]]
        strings = block(strings_section[4], strings_section[5])
        block(section[4], section[5])
        for offset in range(section[4], section[4] + section[5], 16):
            name, address, size, info, _, index = unpack("<IIIBBH", offset)
            name = name_at(strings, name)
            if not name.startswith(function_prefix) or info & 15 != 2 or not size:
                continue
            if index >= len(sections):
                raise ValueError("Invalid P85 function section")
            source = sections[index]
            address &= ~1  # ELF Thumb function marker, if present.
            relative = address - source[3]
            if source[1] != 1 or source[2] & 6 != 6 or relative < 0 or relative + size > source[5]:
                raise ValueError("P85 function is outside executable file data")
            functions[name] = (name, address, block(source[4] + relative, size))
    if not required.issubset(functions) or not rodata:
        raise ValueError("ELF lacks required P85 functions/rodata")
    regions = sorted(functions.values(), key=lambda item: (item[1], item[0])) + rodata
    for name, address, expected in regions:
        if address < base or address + len(expected) > base + SIZE:
            raise ValueError("Reference region is outside the pinned SRAM: " + name)
    return regions


def inspect(elf, snapshot):
    regions = regions_from_elf(elf)
    if len(snapshot) != SIZE:
        raise ValueError("SRAM snapshot must be exactly 1 MiB")
    result = []
    for name, address, expected in regions:
        observed = snapshot[address - BASE:address - BASE + len(expected)]
        result.append({"name": name, "address": hex(address), "bytes": len(expected),
                       "matches": observed == expected,
                       "expected_sha256": digest(expected), "observed_sha256": digest(observed)})
    return {"image_matches": all(item["matches"] for item in result),
            "meaning": "code/rodata presence only; execution requires P85 HELLO and advancing diagnostics",
            "rt0_base": hex(BASE), "sram_bytes": SIZE, "elf_sha256": digest(elf),
            "checked_bytes": sum(item["bytes"] for item in result), "regions": result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path, help="P85 R0 ELF from the exact build")
    parser.add_argument("--snapshot", type=Path, help="Offline full 1 MiB RT0 SRAM dump")
    args = parser.parse_args()
    try:
        elf = args.elf.read_bytes()
        # Validate all reference ranges before opening the board memory device.
        regions_from_elf(elf)
        if args.snapshot:
            snapshot = args.snapshot.read_bytes()
        else:
            fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
            try:
                with mmap.mmap(fd, SIZE, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ, offset=BASE) as mapped:
                    snapshot = mapped[:]
            finally:
                os.close(fd)
        report = inspect(elf, snapshot)
        if not args.snapshot:
            report["boot_id"] = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        print(json.dumps(report, indent=2))
        return 0 if report["image_matches"] else 1
    except (OSError, ValueError, struct.error) as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
