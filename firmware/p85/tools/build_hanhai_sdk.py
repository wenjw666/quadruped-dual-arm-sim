#!/usr/bin/env python3
"""Stage, build and collect full P85 Hanhai R0 firmware on Linux/WSL; never flash."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from prepare_hanhai_sdk import prepare, P85, LOCK


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk', required=True, type=Path)
    parser.add_argument('--work', required=True, type=Path, help='New isolated build directory')
    parser.add_argument('--artifacts', required=True, type=Path, help='New artifact directory')
    parser.add_argument('--toolchain-bin', required=True, type=Path)
    parser.add_argument('--make-dir', type=Path, help='Optional directory containing make')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument("--boot-loopback", action="store_true", help="Build the startup internal CAN0 loopback variant")
    parser.add_argument("--boot-can-link", action="store_true", help="Build startup CAN0/CAN1 physical test (also runs CAN0 internal loopback)")
    args = parser.parse_args()
    if os.name != 'posix':
        parser.error('Run this script inside Linux/WSL (SDK host tools are Linux x86-64)')
    work, artifacts = args.work.resolve(), args.artifacts.resolve()
    sdk = args.sdk.resolve()
    if args.jobs < 1 or work.exists() or artifacts.exists():
        parser.error('jobs must be positive; work/artifacts must be NEW directories')
    # Avoid placing generated output inside sources or inside each other.
    for a, b in ((work, artifacts), (work, sdk), (artifacts, sdk)):
        if a == b or a in b.parents or b in a.parents:
            parser.error('SDK, work and artifacts must be separate directories')
    env = os.environ.copy()
    env['PATH'] = os.pathsep.join(str(p.resolve()) for p in
        [args.toolchain_bin] + ([args.make_dir] if args.make_dir else [])) + os.pathsep + env.get('PATH', '')
    for command in ('bash', 'make', 'git', 'python3', 'arm-none-eabi-gcc',
                    'arm-none-eabi-size', 'arm-none-eabi-readelf', 'arm-none-eabi-nm'):
        if not shutil.which(command, path=env['PATH']):
            parser.error('Missing tool: ' + command)
    artifacts.mkdir(parents=True)
    report = {'status': 'building', 'started_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'Full R0 firmware compilation/sign-package generation; no flashing or hardware validation',
              'boot_loopback': args.boot_loopback or args.boot_can_link, 'boot_can_link': args.boot_can_link,
              'sdk_lock': LOCK, 'sources_sha256': {}, 'artifacts': {}}
    for directory in ('src', 'include', 'rt0', 'tools'):
        for p in sorted((P85 / directory).rglob('*')):
            if p.suffix in ('.c', '.h', '.py', '.json'):
                report['sources_sha256'][p.relative_to(P85).as_posix()] = digest(p)

    def run(command):
        return subprocess.check_output(command, cwd=work, env=env,
            text=True, encoding='utf-8', errors='replace', stderr=subprocess.STDOUT)

    try:
        work.mkdir(parents=True)
        realtime = work / 'realtime'
        prepare(sdk, realtime, args.boot_loopback, args.boot_can_link)
        report['compiler'] = run(['arm-none-eabi-gcc', '--version'])
        if run(['arm-none-eabi-gcc', '-dumpmachine']).strip() != 'arm-none-eabi':
            raise RuntimeError('Unexpected compiler target')
        with (artifacts / 'build.log').open('w', encoding='utf-8') as log:
            result = subprocess.run(['bash', './make.sh', '-j' + str(args.jobs)],
                cwd=realtime, env=env, stdout=log, stderr=subprocess.STDOUT)
        report['build_exit_code'] = result.returncode
        if result.returncode:
            raise RuntimeError('SDK build failed; inspect build.log')
        out = realtime / 'out'
        names = ['realtime_freertos_core0.' + x for x in ('elf', 'map', 'bin', 'hex')]
        names.append('BST-REALTIME0-RELEASE-C1200-1-3-0.bin')
        for name in names:
            source = out / name
            if not source.is_file() or source.stat().st_size == 0:
                raise RuntimeError('Missing/empty build product: ' + name)
            shutil.copy2(source, artifacts / name)
            report['artifacts'][name] = {'bytes': source.stat().st_size, 'sha256': digest(source)}
        elf = out / names[0]
        attrs = run(['arm-none-eabi-readelf', '-h', '-A', '-lW', str(elf)])
        (artifacts / 'elf-attributes.txt').write_text(attrs, encoding='utf-8')
        if 'Tag_CPU_arch_profile: Realtime' not in attrs or 'VFP registers' not in attrs or 'VFPv3-D16' not in attrs:
            raise RuntimeError('Unexpected CPU or float ABI in ELF')
        if not re.search(r'Entry point address:\s+0x4000000\b', attrs):
            raise RuntimeError('Unexpected entry address')
        loads = re.findall(r'^\s*LOAD\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+0x[0-9a-f]+\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)', attrs, re.M | re.I)
        if not loads:
            raise RuntimeError('No ELF LOAD segment')
        end = max(int(address, 16) + int(size, 16) for address, size in loads)
        if min(int(a, 16) for a, _ in loads) != 0x04000000 or end > 0x04100000:
            raise RuntimeError('ELF exceeds reviewed R0 SRAM region')
        report['sram'] = {'base': '0x04000000', 'end_exclusive': hex(end),
                          'used_span_bytes': end - 0x04000000,
                          'capacity_bytes': 0x100000, 'remaining_bytes': 0x04100000 - end}
        report['size_output'] = run(['arm-none-eabi-size', str(elf)])
        symbols = run(['arm-none-eabi-nm', '--defined-only', str(elf)])
        required = ('p85_board_create_tasks', 'p85_hanhai_receive_once', 'p85_runtime_step',
                    'p85_board_rx', 'p85_mcal_send', 'p85_bst_msgbox_open', 'p85_board_status')
        for symbol in required:
            if not re.search(r'\b' + symbol + r'$', symbols, re.M):
                raise RuntimeError('Missing linked P85 symbol: ' + symbol)
        report['linked_p85_symbols'] = required
        signed = report['artifacts'][names[-1]]
        if signed['bytes'] != 0x100000:
            raise RuntimeError('Unexpected signed image slot size')
        logtext = (artifacts / 'build.log').read_text(encoding='utf-8')
        report['warnings'] = sorted(set(line.strip() for line in logtext.splitlines()
            if 'warning:' in line.lower() or 'DeprecationWarning:' in line))
        for name in ('p85.mk', 'p85-sdk-lock.json'):
            shutil.copy2(realtime / name, artifacts / name)
        report['status'] = 'passed'
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        report['status'] = 'failed'
        report['error'] = str(error)
    report['finished_utc'] = datetime.now(timezone.utc).isoformat()
    (artifacts / 'build-report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('P85 full R0 build: ' + report['status'])
    print('Artifacts: ' + str(artifacts))
    if report['status'] != 'passed':
        print(report['error']); return 1
    print(report['size_output'].strip())
    print('SRAM remaining: ' + str(report['sram']['remaining_bytes']) + ' bytes')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
