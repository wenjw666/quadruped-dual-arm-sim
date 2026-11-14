#!/usr/bin/env python3
"""Cross-compile portable P85 objects; does not build bootable R0 firmware."""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

SOURCES = (
    "src/p85_protocol.c",
    "src/p85_control.c",
    "src/p85_wire.c",
    "rt0/p85_runtime.c",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain-bin", type=Path,
                        help="directory containing arm-none-eabi-gcc/ar/readelf/nm")
    parser.add_argument("--output", required=True, type=Path,
                        help="new or empty output directory")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    tools = {}
    for name in ("gcc", "ar", "readelf", "nm"):
        command = "arm-none-eabi-" + name
        executable = shutil.which(command, path=str(args.toolchain_bin.resolve())
                                  if args.toolchain_bin else None)
        if executable is None:
            parser.error(f"missing {command}; provide --toolchain-bin or PATH")
        tools[name] = executable
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error("--output must be a new or empty directory")
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "purpose": "Cortex-R5 portable-object compilation only; no firmware or board execution",
        "commands": [],
    }

    def run(argv: list[str]) -> str:
        process = subprocess.run(argv, cwd=root, text=True, encoding="utf-8",
                                 errors="replace", stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, check=False)
        report["commands"].append({"argv": argv, "returncode": process.returncode,
                                   "output": process.stdout})
        (output / "compile-report.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8")
        if process.returncode:
            raise RuntimeError(f"command failed ({process.returncode}): {argv[0]}\n{process.stdout}")
        return process.stdout

    try:
        report["compiler_version"] = run([tools["gcc"], "--version"])
        target = run([tools["gcc"], "-dumpmachine"]).strip()
        if target != "arm-none-eabi":
            raise RuntimeError(f"unexpected compiler target: {target}")
        # These are probe flags, not a replacement for the vendor SDK ABI flags.
        flags = ["-std=c99", "-mcpu=cortex-r5", "-marm", "-mfloat-abi=soft",
                 "-mlittle-endian", "-ffreestanding", "-fno-common", "-fshort-enums",
                 "-ffunction-sections", "-fdata-sections", "-Os", "-g",
                 "-Wall", "-Wextra", "-Werror", "-Wpedantic", "-Wshadow",
                 "-Wstrict-prototypes", "-I", str(root / "include")]
        report["flags"] = flags
        report["sources"] = {}
        objects = []
        for relative in SOURCES:
            source = root / relative
            report["sources"][relative] = hashlib.sha256(source.read_bytes()).hexdigest()
            obj = output / (source.stem + ".o")
            run([tools["gcc"], *flags, "-c", str(source), "-o", str(obj)])
            objects.append(str(obj))
        archive = output / "libp85_r5.a"
        run([tools["ar"], "rcs", str(archive), *objects])
        attributes = run([tools["readelf"], "-h", "-A", *objects])
        (output / "elf-attributes.txt").write_text(attributes, encoding="utf-8")
        unresolved = run([tools["nm"], "--undefined-only", str(archive)])
        (output / "undefined-symbols.txt").write_text(unresolved, encoding="utf-8")
        report["archive"] = {"bytes": archive.stat().st_size,
                             "sha256": hashlib.sha256(archive.read_bytes()).hexdigest()}
        report["status"] = "passed"
    except (OSError, RuntimeError) as error:
        report["status"] = "failed"
        report["error"] = str(error)
    (output / "compile-report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if report["status"] != "passed":
        print(report["error"])
        return 1
    print(f"Compiled {len(SOURCES)} portable Cortex-R5 objects: {archive}")
    print("This archive is not a signed R0 image and has not run on the board.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
