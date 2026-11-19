#!/usr/bin/env python3
"""Build pinned R2 firmware releasing CAN1 to R0; never flash or reset hardware."""
from __future__ import annotations
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile

P85 = Path(__file__).resolve().parents[1]
HANDOFF = P85 / "rt2/hanhai"
LOCK = json.loads((HANDOFF / "sdk-lock.json").read_text(encoding="utf-8"))

def sha(data):
    return hashlib.sha256(data).hexdigest()

def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError("R2 SDK anchor mismatch: " + old[:100])
    return text.replace(old, new)

def patch(relative, text):
    if relative == "Makefile":
        return once(text, "OBJECTS\t+= $(addprefix $(BUILD)/obj/, $(notdir $(C_SOURCES:%.c=%.o)))",
                    "C_INCLUDES += -I ./p85_handoff\nC_SOURCES += ./p85_handoff/p85_r2_handoff.c\n\n"
                    "OBJECTS\t+= $(addprefix $(BUILD)/obj/, $(notdir $(C_SOURCES:%.c=%.o)))")
    if relative == "make.sh":
        text = once(text, "#!/bin/bash", "#!/bin/bash\nset -e")
        return once(text, "    # 创建/更新版本控制文件", "    if ! command -v repo >/dev/null 2>&1; then\n"
                    "        echo 'SDK revisions recorded in p85-r2-sdk-lock.json'\n        exit 0\n    fi\n\n"
                    "    # 创建/更新版本控制文件")
    if relative.endswith("main.c"):
        return '#include "p85_r2_handoff.h"\n' + once(text, "    Can_Init(&Can_Config);",
                           "    Can_Init(&Can_Config);\n    p85_r2_handoff_init();")
    if relative.endswith("CanApp.c"):
        text = '#include "p85_r2_handoff.h"\n' + once(text,
            "    Can_SetControllerMode(CanController_1, CAN_CS_STARTED);\n"
            "    Can_SetControllerMode(CanController_2, CAN_CS_STARTED);\n"
            "    Can_SetControllerMode(CanController_3, CAN_CS_STARTED);",
            "    /* Hardware CAN1 belongs to R0 in the P85 physical-link variant. */\n"
            "    uint32_t can2_result = Can_SetControllerMode(CanController_2, CAN_CS_STARTED);\n"
            "    uint32_t can3_result = Can_SetControllerMode(CanController_3, CAN_CS_STARTED);\n"
            "    p85_r2_handoff_started(can2_result, can3_result);")
        text = once(text, "            CanAppTick++;", "            CanAppTick++;\n            p85_r2_handoff_tick();")
        return once(text, '    myprintf("send vcan canid:',
                    '    if (bus_id == 1u) {\n'
                    '        myprintf("CAN1 is assigned to P85 on R0\\r\\n");\n'
                    '        return;\n    }\n\n    myprintf("send vcan canid:')
    if relative.endswith("user_cmd_list.c"):
        pattern = r'SHELL_EXPORT_CMD\(SHELL_CMD_PERMISSION\(0\)\|SHELL_CMD_TYPE\(SHELL_TYPE_CMD_MAIN\)\|SHELL_CMD_DISABLE_RETURN,\s*can, shell_can_Test, Can test\);'
        text, count = re.subn(pattern, "/* Raw CAN shell tests disabled while CAN1 belongs to R0. */", text)
        if count != 1:
            raise ValueError("R2 CAN shell export mismatch")
        return text
    if relative.endswith("Can_PBcfg.c"):
        return once(text, "        FLEXCAN_1_BASE,\n        /* Controller Activation */\n        (boolean)TRUE,",
                    "        FLEXCAN_1_BASE,\n        /* Controller Activation: hardware CAN1 is owned by R0. */\n        (boolean)FALSE,")
    if relative.endswith("isr.cfg"):
        return once(text, "\nISR_DEFINE(RT_INTR_LB_RT_LSP0_CAN1_FUNC_IRQ,",
                    "\n/* CAN1 interrupt is installed by R0, never by this R2 variant. */\n//ISR_DEFINE(RT_INTR_LB_RT_LSP0_CAN1_FUNC_IRQ,")
    raise ValueError("Unreviewed R2 patch target: " + relative)

def stage(sources, shared, realtime):
    """Export only pinned Git objects, including separately versioned nested projects."""
    local_names = {"rt-app-core2", "rt-hal-core2", "rt-mcal-cfg-core2", "rt-radar-task"}
    shared_names = {"bsw": shared / "bsw", "os": shared / "bsw/os",
                    "hrt-os-adapt": shared / "rt_hal/config/os_cfg/os_adapt",
                    "rlog": shared / "rt_appl/user/rlog_app"}
    for project in LOCK["projects"]:
        name = project["name"].split("/")[-1]
        source = sources / name if name in local_names else shared_names[name]
        actual = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
        if actual != project["revision"]:
            raise ValueError("R2 source revision mismatch: " + project["name"])
        archive = subprocess.check_output(["git", "-C", str(source), "archive", project["revision"]])
        dest = realtime / project["path"]
        dest.mkdir(parents=True, exist_ok=True)
        with tarfile.open(fileobj=io.BytesIO(archive)) as files:
            # Pinned upstream repositories only; reject paths outside the export root.
            for member in files.getmembers():
                resolved = (dest / member.name).resolve()
                if dest.resolve() not in (resolved, *resolved.parents) or member.issym() or member.islnk():
                    raise ValueError("Unsafe SDK archive entry: " + member.name)
            files.extractall(dest)
    for relative, pin in LOCK["source_sha256_lf"].items():
        path = realtime / relative
        text = path.read_text(encoding="utf-8")
        if sha(text.encode("utf-8")) != pin:
            raise ValueError("R2 source checksum mismatch: " + relative)
        path.write_text(patch(relative, text), encoding="utf-8")
    (realtime / "p85_handoff").mkdir()
    for name in ("p85_r2_handoff.c", "p85_r2_handoff.h"):
        shutil.copy2(HANDOFF / name, realtime / "p85_handoff" / name)
    (realtime / "p85-r2-sdk-lock.json").write_text(json.dumps(LOCK, indent=2) + "\n", encoding="utf-8")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sources", required=True, type=Path)
    parser.add_argument("--shared-sdk", required=True, type=Path)
    parser.add_argument("--work", required=True, type=Path)
    parser.add_argument("--artifacts", required=True, type=Path)
    parser.add_argument("--toolchain-bin", required=True, type=Path)
    parser.add_argument("--jobs", type=int, default=8)
    args = parser.parse_args()
    if os.name != "posix" or args.jobs < 1:
        parser.error("Run on Linux/WSL with a positive job count")
    work, artifacts = args.work.resolve(), args.artifacts.resolve()
    roots = (work, artifacts, args.sources.resolve(), args.shared_sdk.resolve())
    for i, a in enumerate(roots):
        for b in roots[i+1:]:
            if a == b or a in b.parents or b in a.parents:
                parser.error("Build, artifacts and source directories must be separate")
    if work.exists() or artifacts.exists():
        parser.error("Work/artifact directories must be new")
    artifacts.mkdir(parents=True)
    work.mkdir(parents=True)
    report = {"status": "building", "scope": "R2 CAN1 release variant; compilation only", "sdk_lock": LOCK}
    env = os.environ.copy()
    env["PATH"] = str(args.toolchain_bin.resolve()) + os.pathsep + env.get("PATH", "")
    realtime = work / "realtime"
    def run(*command):
        return subprocess.check_output(command, cwd=realtime, env=env, text=True, encoding="utf-8", stderr=subprocess.STDOUT)
    try:
        stage(args.sources.resolve(), args.shared_sdk.resolve(), realtime)
        report["compiler"] = run("arm-none-eabi-gcc", "--version")
        with (artifacts / "build.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(["bash", "./make.sh", "-j"+str(args.jobs)], cwd=realtime, env=env, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError("R2 build failed; inspect build.log")
        out = realtime / "out"
        names = ["realtime_freertos_core2."+ext for ext in ("elf", "bin", "map", "hex")]
        names += ["BST-REALTIME2-RELEASE-C1200-1-3-0.bin"]
        report["artifacts"] = {}
        for name in names:
            data = (out / name).read_bytes()
            if not data:
                raise ValueError("Empty R2 artifact")
            shutil.copy2(out / name, artifacts / name)
            report["artifacts"][name] = {"bytes": len(data), "sha256": sha(data)}
        attributes = run("arm-none-eabi-readelf", "-h", "-A", "-lW", str(out / names[0]))
        (artifacts / "elf-attributes.txt").write_text(attributes, encoding="utf-8")
        if not re.search(r"Entry point address:\s+0x4200000\b", attributes):
            raise ValueError("Unexpected R2 entry point")
        if "Tag_CPU_arch_profile: Realtime" not in attributes or "VFP registers" not in attributes:
            raise ValueError("Unexpected R2 ABI")
        loads = re.findall(r"^\s*LOAD\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+0x[0-9a-f]+\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)", attributes, re.M | re.I)
        end = max(int(a,16)+int(s,16) for a,s in loads)
        if min(int(a,16) for a,_ in loads) != 0x04200000 or end > 0x04300000:
            raise ValueError("R2 SRAM range mismatch")
        report["sram"] = {"base": "0x04200000", "end_exclusive": hex(end), "remaining_bytes": 0x04300000-end}
        symbols = run("arm-none-eabi-nm", "--defined-only", str(out / names[0]))
        for name in ("p85_r2_handoff_init", "p85_r2_handoff_tick", "p85_r2_handoff_report"):
            if not re.search(r"\b"+name+r"$", symbols, re.M):
                raise ValueError("Missing R2 handoff symbol: "+name)
        if report["artifacts"][names[-1]]["bytes"] != 0x100000:
            raise ValueError("Unexpected R2 signed slot size")
        report["size_output"] = run("arm-none-eabi-size", str(out / names[0]))
        report["warnings"] = sorted(set(l.strip() for l in (artifacts / "build.log").read_text(encoding="utf-8").splitlines() if "warning:" in l.lower()))
        report["status"] = "passed"
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        report.update(status="failed", error=str(error))
    (artifacts / "build-report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print("R2 CAN1 release build: "+report["status"])
    print(report.get("error", report.get("size_output", "")))
    return 0 if report["status"] == "passed" else 1

if __name__ == "__main__":
    raise SystemExit(main())
