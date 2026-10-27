"""Stop this workspace's selected backend and auxiliary viewers, never Docker."""
import argparse
import os
import signal
import time
from pathlib import Path
parser = argparse.ArgumentParser()
parser.add_argument('--backend', choices=['gazebo','mujoco','all'], default='gazebo')
parser.add_argument('--check', action='store_true')
parser.add_argument('--viewer-only', action='store_true')
options = parser.parse_args()
targets = []
identities = {}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit() or int(proc.name)==os.getpid(): continue
    try:
        args = proc.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
        env = dict(s.split('=',1) for s in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in s)
    except (OSError, UnicodeError): continue
    is_launch = any(args[i:i+3]==['launch','qda_bringup','pick_place.launch.py'] for i in range(len(args)))
    is_rviz = any(Path(a).name=='rviz2' for a in args) and any('factory.rviz' in a for a in args)
    if options.viewer_only and not is_rviz: continue
    backend = 'mujoco' if 'backend:=mujoco' in args else 'gazebo'
    auxiliary = (bool(args and Path(args[0]).name=='gzclient') or
                 is_rviz or
                 any(a.endswith('/visualization_node.py') for a in args) or
                 any(a in ('/ws/install/qda_bringup/lib/qda_bringup/visualization_node',
                           '/ws/install/qda_brain/lib/qda_brain/skill_server') for a in args))
    if auxiliary: backend = 'mujoco' if env.get('ROS_DOMAIN_ID')=='8' else 'gazebo'
    if (is_launch or auxiliary) and options.backend in ('all',backend):
        targets.append(int(proc.name));identities[int(proc.name)]=args
if options.check:
    print('running' if targets else 'stopped')
    raise SystemExit(0 if targets else 1)
for pid in targets:
    try:
        os.kill(pid,signal.SIGINT)
        print(f'Sent SIGINT to simulation process {pid}')
    except ProcessLookupError: pass
deadline=time.monotonic()+20
while time.monotonic()<deadline and any(Path(f'/proc/{p}').exists() for p in targets):time.sleep(.2)
def still_owned(pid):
    try:return Path(f'/proc/{pid}/cmdline').read_bytes().decode().strip('\0').split('\0')==identities[pid]
    except (OSError,UnicodeError):return False
# rclpy executor shutdown can wait forever on an action whose server has stopped.
# Only escalate the exact command identities selected above, never other nodes.
for sig,grace in ((signal.SIGTERM,3.),(signal.SIGKILL,1.)):
    remaining=[pid for pid in targets if still_owned(pid)]
    if not remaining:break
    for pid in remaining:
        try:os.kill(pid,sig);print(f'Sent {sig.name} to stopped-but-unresponsive simulation process {pid}')
        except ProcessLookupError:pass
    until=time.monotonic()+grace
    while time.monotonic()<until and any(still_owned(pid) for pid in remaining):time.sleep(.1)
if not targets:print('No matching simulation launch/viewer is running.')
