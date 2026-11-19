#!/usr/bin/env python3
"""Read a completed CAN0/CAN1 physical test report from verified R0 SRAM."""
import argparse
import json
import mmap
import os
from pathlib import Path
import struct
import sys
from inspect_can_loopback import report_address, LENGTHS, REGS
from inspect_r0_image import BASE, SIZE, inspect
FIELDS = ("magic version state failure phase requested_per_direction elapsed_ms restored "
          "bad_metadata bad_payload unexpected timeouts").split()
WORDS = 100

def decode(data):
    if len(data)!=WORDS*4:
        raise ValueError("Wrong physical test report size")
    w=struct.unpack("<"+"I"*WORDS,data)
    r=dict(zip(FIELDS,w[:12]));r["faults"]={"can0":w[12],"can1":w[13]}
    r["directions"]={}
    for d,label in enumerate(("can0_to_can1","can1_to_can0")):
        i=14+d*19
        r["directions"][label]=dict(zip(("submitted","confirmed","received"),w[i:i+3]))
        r["directions"][label]["length_counts"]=dict(zip(map(str,LENGTHS),w[i+3:i+19]))
    for p,phase in enumerate(("before","tested","after")):
        r[phase]={}
        for c in range(2):
            i=52+p*16+c*8
            r[phase]["can"+str(c)]=dict(zip(REGS,w[i:i+8]))
    directions=list(r["directions"].values())
    all_regs=[v for phase in ("before","tested","after") for v in r[phase].values()]
    on_bus=[v for phase in ("before","tested") for v in r[phase].values()]
    checks={
        "report_complete":r["magic"]==0x504c4b31 and r["version"]==1 and r["state"]==2 and r["phase"]==2,
        "no_failures":all(r[k]==0 for k in ("failure","bad_metadata","bad_payload","unexpected","timeouts")) and not any(r["faults"].values()),
        "both_directions":r["requested_per_direction"]==512 and all(all(d[k]==512 for k in ("submitted","confirmed","received")) for d in directions),
        "all_lengths":all(all(n==32 for n in d["length_counts"].values()) for d in directions),
        "normal_no_self_reception":all(not (v["CTRL1"]&0x1008) and bool(v["MCR"]&0x20000) for v in all_regs),
        "both_controllers_active_during_test":all(not(v["MCR"]&0x91000000) for v in on_bus),
        "target_rates":all(v["CBT"]==0x80139a73 and v["FDCBT"]==0x31c63 and not(v["CTRL2"]&0x2000) for v in all_regs),
        "fd_brs_tdc":all(v["MCR"]&0x800 and (v["FDCTRL"]&0x80009f00)==0x80008c00 for v in all_regs),
        "no_error_counters":all(v["ECR"]==0 for v in on_bus) and all(not(v["FDCTRL"]&0x4000) for v in on_bus),
        "settings_restored":r["restored"]==1 and all((r["before"][c]["FDCTRL"]&0xffffbfc0)==(r["after"][c]["FDCTRL"]&0xffffbfc0) for c in ("can0","can1")),
        "can0_active_can1_stopped":not(r["after"]["can0"]["MCR"]&0x91000000) and bool(r["after"]["can1"]["MCR"]&0x10000000),
        "elapsed_time":2048<=r["elapsed_ms"]<=120000,
    }
    r["checks"]=checks;r["status"]="passed" if all(checks.values()) else "failed"
    return r

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf",required=True,type=Path)
    parser.add_argument("--snapshot",type=Path)
    args=parser.parse_args()
    try:
        elf=args.elf.read_bytes()
        address=report_address(elf,b"p85_can_link_report",WORDS)
        boot=None if args.snapshot else Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        if args.snapshot:
            snapshot=args.snapshot.read_bytes()
        else:
            fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
            try:
                with mmap.mmap(fd,SIZE,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=BASE) as mem:
                    snapshot=mem[:];second=mem[:]
                    a=address-BASE
                    if snapshot[a:a+WORDS*4]!=second[a:a+WORDS*4]:
                        raise ValueError("Physical test report is still changing")
            finally:
                os.close(fd)
        identity=inspect(elf,snapshot)
        if not identity["image_matches"]:
            raise ValueError("SRAM code/rodata does not match test ELF")
        r=decode(snapshot[address-BASE:address-BASE+WORDS*4])
        r.update(address=hex(address),elf_sha256=identity["elf_sha256"],image_matches=True,
                 checked_bytes=identity["checked_bytes"],boot_id=boot)
        if boot and Path("/proc/sys/kernel/random/boot_id").read_text().strip()!=boot:
            raise ValueError("Boot changed during read")
        print(json.dumps(r,indent=2));return 0 if r["status"]=="passed" else 1
    except (OSError,ValueError,struct.error,IndexError) as e:
        print(json.dumps({"status":"failed","error":str(e)}));return 2
if __name__=="__main__":
    sys.exit(main())
