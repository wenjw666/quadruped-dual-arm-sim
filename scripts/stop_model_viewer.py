import os,signal
from pathlib import Path
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:args=p.joinpath('cmdline').read_bytes().decode().split('\0')
 except (OSError,UnicodeError):continue
 if '/ws/scripts/robot_model_viewer.py' in args:
  try:os.kill(int(p.name),signal.SIGINT)
  except ProcessLookupError:pass
