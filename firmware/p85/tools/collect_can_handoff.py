#!/usr/bin/env python3
"""Preserve one diagnostic boot from R0/R2 SRAM, inspect it, restore both slots.
No motor commands, CAN MMIO reads, acceptance or reboot operations are provided.
"""
import argparse
import hashlib
import json
import mmap
import os
from pathlib import Path
import subprocess
import sys
import time
import can_handoff_flash_guard as guard
from inspect_can_loopback import report_address, WORDS as LOOPBACK_WORDS
from inspect_can_link import WORDS as LINK_WORDS
from inspect_can_trace import WORDS as TRACE_WORDS
BASE = Path(__file__).resolve().parent

def snapshot(address, twice=False):
    fd=os.open("/dev/mem",os.O_RDONLY|os.O_SYNC)
    try:
        with mmap.mmap(fd,0x100000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=address) as mem:
            first=mem[:]
            if twice:time.sleep(0.25)
            second=mem[:]
        return first,second
    finally:
        os.close(fd)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only",action="store_true",help="Capture/inspect only, without any Flash action")
    args=parser.parse_args()
    boot=guard.boot_id()
    # Never collect/restore the live old image as the next diagnostic boot.
    if not args.check_only:
        armed=json.loads((BASE/"armed.json").read_text(encoding="utf-8"))
        if armed["origin_boot_id"]==boot:
            raise RuntimeError("Handoff collector refuses origin boot")
    prefix="check-only-" if args.check_only else "diagnostic-"
    target=BASE/(prefix+boot+".json")
    result={"boot_id":boot,"accepted":False,"diagnostic_only":True,"check_only":args.check_only,"checks":{}}
    def save():
        temp=target.with_suffix(".tmp")
        temp.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
        temp.replace(target)
    def run(label, command, timeout=30):
        p=subprocess.run(command,capture_output=True,text=True,encoding="utf-8",timeout=timeout)
        result["checks"][label]={"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}
        save()
        return p.returncode
    try:
        guard.metadata()
        uptime=float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0])
        if uptime<5:time.sleep(5-uptime)
        data,second=snapshot(0x04000000)
        elf=(BASE/"realtime_freertos_core0.elf").read_bytes()
        for symbol,words in ((b"p85_can_loopback_report",LOOPBACK_WORDS),(b"p85_can_link_report",LINK_WORDS),(b"p85_can_trace_report",TRACE_WORDS)):
            a=report_address(elf,symbol,words)-0x04000000
            if data[a:a+words*4]!=second[a:a+words*4]:
                raise RuntimeError("R0 published report changed during snapshot: "+symbol.decode())
        r0file=BASE/(prefix+"r0-"+boot+".bin")
        r0file.write_bytes(data)
        r2first,r2second=snapshot(0x04200000,True)
        r2files=[BASE/(prefix+"r2-"+suffix+"-"+boot+".bin") for suffix in ("first","second")]
        for p,d in zip(r2files,(r2first,r2second)):p.write_bytes(d)
        result["snapshots"]={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (r0file,*r2files)}
        statuses=[]
        for label,tool in (("image","inspect_r0_image.py"),("loopback","inspect_can_loopback.py"),("link","inspect_can_link.py"),("trace","inspect_can_trace.py")):
            statuses.append(run(label,[sys.executable,str(BASE/tool),"--elf",str(BASE/"realtime_freertos_core0.elf"),"--snapshot",str(r0file)]))
        r2code=run("r2_handoff",[sys.executable,str(BASE/"inspect_r2_handoff.py"),"--elf",str(BASE/"realtime_freertos_core2.elf"),"--snapshot",str(r2files[0]),"--second-snapshot",str(r2files[1])])
        statuses.append(r2code)
        result["r2_handoff_runtime_verified"]=r2code==0
        statuses.append(run("aon",[sys.executable,str(BASE/"inspect_aon_state.py")]))
        if guard.boot_id()!=boot:raise RuntimeError("Boot changed during collection")
        result["collection_status"]="complete"
        result["all_report_exit_codes_zero"]=not any(statuses)
    except Exception as error:
        result.update(collection_status="failed",error=str(error))
    finally:
        save()
        if not args.check_only:
            code=run("restore",[sys.executable,str(BASE/"can_handoff_flash_guard.py"),"restore"],240)
            result["baseline_restored"]=code==0
        result["uptime_end_s"]=float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0])
        save()
    print(json.dumps({k:result[k] for k in ("boot_id","accepted","collection_status","check_only")}))
    return 0 if result.get("baseline_restored",args.check_only) and result["collection_status"]=="complete" else 1
if __name__=="__main__":
    sys.exit(main())
