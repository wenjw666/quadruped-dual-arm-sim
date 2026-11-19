#!/usr/bin/env python3
"""Read the startup CAN0 hardware loopback report. No MMIO or motor writes.
An exact ELF code/rodata match is required before interpreting the report.
"""
import argparse
import json
import mmap
import os
from pathlib import Path
import struct
import sys
from inspect_r0_image import BASE, SIZE, inspect

FIELDS = ("magic version state failure requested submitted confirmed received "
          "bad_metadata bad_payload unexpected timeouts faults elapsed_ms restored").split()
LENGTHS = (0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64)
REGS = ("MCR", "CTRL1", "CBT", "FDCBT", "FDCTRL", "CTRL2", "ESR1", "ECR")
WORDS = len(FIELDS) + len(LENGTHS) + 3 * len(REGS)

def report_address(elf, symbol=b"p85_can_loopback_report", words=WORDS, *, base=BASE):
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", elf)
    if header[0][:7] != b"\x7fELF\x01\x01\x01" or header[1:3] != (2,40):
        raise ValueError("Expected ELF32 little-endian ARM executable")
    sections = [struct.unpack_from("<10I", elf, header[6]+i*40) for i in range(header[12])]
    found = []
    for section in sections:
        if section[1] != 2:
            continue
        if section[9] != 16 or section[5] % 16:
            raise ValueError("Invalid ELF symbols")
        strings_section = sections[section[6]]
        strings = elf[strings_section[4]:strings_section[4]+strings_section[5]]
        for off in range(section[4], section[4]+section[5], 16):
            name, address, size, info, _, index = struct.unpack_from("<IIIBBH", elf, off)
            end = strings.find(b"\0", name)
            if strings[name:end] != symbol:
                continue
            if info & 15 != 1 or size != words*4 or index >= len(sections):
                raise ValueError("Unexpected report object type/size/section")
            source = sections[index]
            if source[2] & 3 != 3 or not (base <= address < address+size <= base+SIZE):
                raise ValueError("Report is outside writable SRAM")
            found.append(address)
    if len(found) != 1:
        raise ValueError("Expected one loopback report symbol")
    return found[0]

def decode(data):
    if len(data) != WORDS*4:
        raise ValueError("Wrong loopback report size")
    words = struct.unpack("<"+"I"*WORDS, data)
    result = dict(zip(FIELDS, words))
    result["length_counts"] = dict(zip(map(str,LENGTHS), words[15:31]))
    for i, phase in enumerate(("before", "loopback", "after")):
        result[phase] = dict(zip(REGS,words[31+i*8:39+i*8]))
    checks = {
        "report_complete": result["magic"]==0x504c4231 and result["version"]==1 and result["state"]==2,
        "no_failures": all(result[k]==0 for k in ("failure","bad_metadata","bad_payload","unexpected","timeouts","faults")),
        "all_frames": all(result[k]==512 for k in ("requested","submitted","confirmed","received")),
        "all_lengths": all(n==32 for n in result["length_counts"].values()),
        "mode_restored": result["restored"]==1,
        "elapsed_time": 1024 <= result["elapsed_ms"] <= 60000,
    }
    before, loop, after = (result[k] for k in ("before","loopback","after"))
    # LPB=CTRL1[12], LOM=CTRL1[3], SRXDIS=MCR[17].
    checks["hardware_loopback"] = bool(loop["CTRL1"] & 0x1000) and not (loop["CTRL1"] & 8 or loop["MCR"] & 0x20000)
    checks["normal_before_after"] = all(not (r["CTRL1"] & 0x1008) and bool(r["MCR"] & 0x20000) for r in (before,after))
    checks["target_rates"] = all(r["CBT"]==0x80139a73 and r["FDCBT"]==0x00031c63 for r in (before,loop,after))
    # TDC is disabled only during internal loopback; preserve all other FD
    # configuration and restore the complete production settings afterwards.
    # Ignore the measured TDCVAL and sticky TDCFAIL when comparing settings.
    mask = 0xffffbfc0
    checks["loopback_tdc_disabled"] = not bool(loop["FDCTRL"] & 0x8000)
    checks["loopback_fd_settings"] = (loop["FDCTRL"] & mask) == (before["FDCTRL"] & mask & ~0x8000)
    checks["fd_settings_restored"] = (after["FDCTRL"] & mask) == (before["FDCTRL"] & mask)
    result["checks"] = checks
    result["status"] = "passed" if all(checks.values()) else "failed"
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True,type=Path)
    parser.add_argument("--snapshot",type=Path)
    args=parser.parse_args()
    try:
        elf=args.elf.read_bytes()
        address=report_address(elf)
        boot=None if args.snapshot else Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        if args.snapshot:
            snapshot=args.snapshot.read_bytes()
        else:
            fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
            try:
                with mmap.mmap(fd,SIZE,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=BASE) as mem:
                    snapshot=mem[:]
                    second = mem[:]  # Copy from aligned mapping; never slice device memory at a symbol offset.
                    if snapshot[address-BASE:address-BASE+WORDS*4] != second[address-BASE:address-BASE+WORDS*4]:
                        raise ValueError("Loopback report is still changing")
            finally:
                os.close(fd)
        identity=inspect(elf,snapshot)
        if not identity["image_matches"]:
            raise ValueError("SRAM code/rodata does not match test ELF")
        result=decode(snapshot[address-BASE:address-BASE+WORDS*4])
        result.update(address=hex(address),elf_sha256=identity["elf_sha256"],image_matches=True,checked_bytes=identity["checked_bytes"],boot_id=boot)
        if boot and Path("/proc/sys/kernel/random/boot_id").read_text().strip()!=boot:
            raise ValueError("Boot changed during read")
        print(json.dumps(result,indent=2))
        return 0 if result["status"]=="passed" else 1
    except (OSError,ValueError,struct.error,IndexError) as error:
        print(json.dumps({"status":"failed","error":str(error)}))
        return 2

if __name__=="__main__":
    sys.exit(main())
