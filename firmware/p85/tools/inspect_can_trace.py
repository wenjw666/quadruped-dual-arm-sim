#!/usr/bin/env python3
"""Decode startup-only R0 CAN diagnostics from verified SRAM (never CAN MMIO)."""
import argparse
import json
import mmap
import os
from pathlib import Path
import struct
import sys
from inspect_can_loopback import report_address, REGS
from inspect_r0_image import BASE, SIZE, inspect
CAPACITY=96
WORDS=8+16*CAPACITY
STAGES={1:"init_enter",2:"freeze",3:"disable",4:"soft_reset",5:"rx_fifo",
        6:"max_message_buffers",7:"init_return",8:"start_enter",
        9:"start_return",10:"finish",11:"exit_freeze"}

def decode(data):
    if len(data)!=WORDS*4:
        raise ValueError("Wrong CAN trace size")
    w=struct.unpack("<"+"I"*WORDS,data)
    magic,version,count,dropped,finalized,failure,capacity,entry_words=w[:8]
    if magic!=0x50545231 or version!=1 or capacity!=CAPACITY or entry_words!=16:
        raise ValueError("Unrecognized CAN trace layout")
    if count>CAPACITY or finalized not in (0,1):
        raise ValueError("Invalid CAN trace header")
    events=[]
    for i in range(count):
        a=8+i*16
        stage,instance,status,aux=w[a:a+4]
        if stage not in STAGES or instance>1 or any(w[a+12:a+16]):
            raise ValueError("Invalid CAN trace entry")
        events.append({"index":i,"stage":STAGES[stage],"instance":instance,
                       "status":status,"aux":aux,"registers":dict(zip(REGS,w[a+4:a+12]))})
    if finalized and (not events or events[-1]["stage"]!="finish" or events[-1]["status"]!=failure):
        raise ValueError("Final CAN trace marker is inconsistent")
    errors=[e for e in events if e["status"] and e["stage"] not in ("init_enter","start_enter","finish")]
    return {"status":"complete" if finalized and not dropped else "incomplete",
            "failure":failure,"dropped":dropped,"finalized":bool(finalized),
            "event_count":count,"errors":errors,"events":events,
            "can1_ownership_confirmed":False}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf",required=True,type=Path)
    parser.add_argument("--snapshot",type=Path)
    args=parser.parse_args()
    try:
        elf=args.elf.read_bytes()
        address=report_address(elf,b"p85_can_trace_report",WORDS)
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
                        raise ValueError("CAN trace is still changing")
            finally:
                os.close(fd)
        identity=inspect(elf,snapshot)
        if not identity["image_matches"]:
            raise ValueError("SRAM code/rodata does not match trace ELF")
        r=decode(snapshot[address-BASE:address-BASE+WORDS*4])
        r.update(address=hex(address),elf_sha256=identity["elf_sha256"],image_matches=True,
                 checked_bytes=identity["checked_bytes"],boot_id=boot)
        if boot and Path("/proc/sys/kernel/random/boot_id").read_text().strip()!=boot:
            raise ValueError("Boot changed during read")
        print(json.dumps(r,indent=2));return 0 if r["status"]=="complete" else 1
    except (OSError,ValueError,struct.error,IndexError) as e:
        print(json.dumps({"status":"failed","error":str(e)}));return 2
if __name__=="__main__":
    sys.exit(main())
