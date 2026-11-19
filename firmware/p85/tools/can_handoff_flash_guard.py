#!/usr/bin/env python3
"""One-boot R0/R2 CAN1 handoff diagnostic. Only the two fixed RT slots may change.
Rendered packages pin package-info.json. No acceptance or reboot operation exists.
"""
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
BASE = Path(__file__).resolve().parent
EXPECTED = Path("/var/lib/p85-rt0-deploy/20261117-can-handoff-01")
PACKAGE_SHA256 = "__PACKAGE_SHA256__"
OLD_FULL = "235ebdfd03cb5be6dd19dd5eb986dcec521948ba105f1d97a911cbb8d1eb302e"
TOTAL, SLOT = 0x4000000, 0x100000
# Write R2 first: a power loss between slots leaves CAN1 released and baseline R0.
SLOTS = ((2, 0xD00000), (0, 0xB00000))
DEVICE = "/dev/mtdblock0"
MARKER = BASE / "armed.json"
TIMER = "p85-can-handoff-rollback.timer"
COLLECTOR = "p85-can-handoff-collect.service"

def sha(data):
    return hashlib.sha256(data).hexdigest()

def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()

def run(*command):
    print("+", *command, flush=True)
    subprocess.run(command, check=True, timeout=180)

def emit(name, data):
    path = BASE / name
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
    temp.replace(path)
    print(json.dumps(data), flush=True)

def baseline():
    data = (BASE / "flash-before.bin").read_bytes()
    if len(data) != TOTAL or sha(data) != OLD_FULL:
        raise RuntimeError("Baseline full-flash size/hash mismatch")
    for core, offset in SLOTS:
        (BASE / f"rt{core}-before.bin").write_bytes(data[offset:offset+SLOT])
    return data

def metadata():
    raw = (BASE / "package-info.json").read_bytes()
    if sha(raw) != PACKAGE_SHA256:
        raise RuntimeError("Pinned handoff package metadata mismatch")
    info = json.loads(raw)
    if set(info["candidates"]) != {"0", "2"}:
        raise RuntimeError("Candidate map must contain exactly R0 and R2")
    for name in info["candidates"].values():
        if not isinstance(name, str) or Path(name).name != name or name not in info["pinned"]:
            raise RuntimeError("Candidate must be a pinned package basename")
    for name, pin in info["pinned"].items():
        if Path(name).name != name or sha((BASE / name).read_bytes()) != pin:
            raise RuntimeError("Pinned package file mismatch: " + name)
    for core, _ in SLOTS:
        data = (BASE / info["candidates"][str(core)]).read_bytes()
        if len(data) != SLOT:
            raise RuntimeError("Unexpected candidate slot size")
        magic, payload, marker, siglen = struct.unpack_from("<4I", data)
        if magic != 0xABCD1234 or marker != 0xffffffff or siglen != 64 or not 32 < payload < SLOT-256:
            raise RuntimeError("Unexpected candidate signed package header")
    return info

def read_flash(name):
    path = BASE / name
    run("mtd_debug", "read", DEVICE, "0", hex(TOTAL), str(path))
    data = path.read_bytes()
    if len(data) != TOTAL:
        raise RuntimeError("Short full-flash read")
    return data

def outside_unchanged(current, original):
    if len(current) != TOTAL or len(original) != TOTAL:
        raise RuntimeError("Full-flash size mismatch")
    cursor = 0
    for _, offset in sorted(SLOTS, key=lambda item:item[1]):
        if current[cursor:offset] != original[cursor:offset]:
            raise RuntimeError("Flash changed outside the R0/R2 diagnostic slots")
        cursor = offset+SLOT
    if current[cursor:] != original[cursor:]:
        raise RuntimeError("Flash changed outside the R0/R2 diagnostic slots")

def write_slot(core, offset, path, expected):
    if (core,offset) not in SLOTS or len(expected) != SLOT or path.read_bytes() != expected:
        raise RuntimeError("Refusing unverified slot input")
    run("mtd_debug", "erase", DEVICE, hex(offset), hex(SLOT))
    run("mtd_debug", "write", DEVICE, hex(offset), hex(SLOT), str(path))
    run("sync")

def disarm():
    # Keep the recovery marker until fallback and collector are disabled.
    run("systemctl", "disable", "--now", TIMER)
    run("systemctl", "disable", COLLECTOR)
    MARKER.unlink(missing_ok=True)
    run("sync")

def inspect():
    original, info = baseline(), metadata()
    if boot_id() != info["origin_boot_id"]:
        raise RuntimeError("Boot changed since package preparation")
    if read_flash("flash-preflight.bin") != original:
        raise RuntimeError("Live full-flash differs from accepted baseline")
    emit("preflight.json", {"status":"passed", "boot_id":boot_id(), "baseline_sha256":sha(original),
         "slots":{str(c):hex(o) for c,o in SLOTS}, "diagnostic_only":True})
    return original, info

def install():
    if MARKER.exists():
        raise RuntimeError("Handoff transaction already armed")
    original, info = inspect()
    wanted = bytearray(original)
    for core, offset in SLOTS:
        wanted[offset:offset+SLOT] = (BASE / info["candidates"][str(core)]).read_bytes()
    emit("armed.json", {"origin_boot_id":info["origin_boot_id"], "baseline_sha256":sha(original),
         "expected_full_sha256":sha(wanted), "accepted":False})
    try:
        run("systemctl", "enable", TIMER, COLLECTOR)
        run("sync")
        for core, offset in SLOTS:
            if boot_id() != info["origin_boot_id"]:
                raise RuntimeError("Boot changed before slot write")
            source = BASE / info["candidates"][str(core)]
            write_slot(core, offset, source, wanted[offset:offset+SLOT])
        current = read_flash("flash-after.bin")
        outside_unchanged(current, original)
        if current != wanted:
            raise RuntimeError("Dual-slot full-flash readback mismatch")
        emit("install-result.json", {"status":"written-and-verified", "full_flash_sha256":sha(current),
             "outside_r0_r2_unchanged":True, "diagnostic_only":True, "accepted":False, "reboot_issued":False})
    except Exception:
        restore(False)
        raise

def restore(automatic=False):
    # Recovery deliberately requires only the independently pinned baseline.
    original = baseline()
    if MARKER.exists():
        armed = json.loads(MARKER.read_text(encoding="utf-8"))
        if automatic and armed["origin_boot_id"] == boot_id():
            raise RuntimeError("Automatic rollback refuses the deployment origin boot")
    else:
        if read_flash("flash-unarmed-check.bin") != original:
            raise RuntimeError("Unarmed transaction with non-baseline Flash; refusing writes")
        disarm()
        return
    current = read_flash("flash-before-rollback.bin")
    outside_unchanged(current, original)
    for core, offset in SLOTS:
        expected = original[offset:offset+SLOT]
        if current[offset:offset+SLOT] != expected:
            write_slot(core, offset, BASE / f"rt{core}-before.bin", expected)
    restored = read_flash("flash-restored.bin")
    if restored != original:
        raise RuntimeError("Dual-slot baseline restore verification failed")
    emit("rollback-result.json", {"status":"original-restored", "sha256":sha(restored),
         "boot_id":boot_id(), "reboot_issued":False, "accepted":False})
    disarm()

def main():
    import fcntl
    if os.geteuid() != 0 or BASE != EXPECTED or PACKAGE_SHA256.startswith("__"):
        sys.exit("Run the rendered pinned guard as root from its deployment directory")
    with (BASE / "transaction.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        command = sys.argv[1] if len(sys.argv) == 2 else ""
        if command == "inspect": inspect()
        elif command == "install": install()
        elif command == "rollback": restore(True)
        elif command == "restore": restore(False)
        else: sys.exit("Expected inspect/install/rollback/restore; diagnostic acceptance disabled")
if __name__ == "__main__":
    main()
