"""Gracefully stop only the recorder owning the exact requested take path."""
import argparse
import os
import signal
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--take',required=True);a=p.parse_args()
target=Path(a.take).resolve();stopped=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
    try:
        args=entry.joinpath('cmdline').read_bytes().decode().split('\0')
        if '/ws/scripts/record_product_demo.py' not in args or '--out' not in args:continue
        if Path(args[args.index('--out')+1]).resolve()!=target:continue
        os.kill(int(entry.name),signal.SIGINT);stopped.append(int(entry.name))
    except (OSError,UnicodeError,IndexError):continue
print({'stopped_recorders':stopped,'take':str(target)})
