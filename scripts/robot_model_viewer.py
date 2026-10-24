"""Interactive isolated URDF model review. O=open, C=close, R=reset view."""
import time,json
from pathlib import Path
import numpy as np
import mujoco
from mujoco import viewer
root=Path('/ws/output/model_refinement_v1')
m=mujoco.MjModel.from_xml_path(str(root/'studio.xml'));d=mujoco.MjData(m)
d.qpos[:]=np.load(root/'review_qpos.npy')
manifest=json.loads((root/'studio.manifest.json').read_text())
command={'opening':.15,'reset':False}
def key(code):
 if code==79:command['opening']=0.
 if code==67:command['opening']=.8
 if code==82:command['reset']=True
with viewer.launch_passive(m,d,key_callback=key) as v:
 v.opt.geomgroup[3]=1
 def reset():
  v.cam.lookat[:]=[0,0,.80];v.cam.distance=2.35;v.cam.azimuth=145;v.cam.elevation=-18
 reset()
 print('MODEL_REVIEW_READY. Mouse: orbit/zoom. O=open; C=close; R=reset view.',flush=True)
 while v.is_running():
  for side in ('left','right'):
   for finger in ('left','right'):
    a=m.jnt_qposadr[m.joint(f'{side}_finger_{finger}_joint').id]
    d.qpos[a]+=np.clip(command['opening']-d.qpos[a],-.012,.012)
  for name,mm in manifest['mimics'].items():
   a=m.jnt_qposadr[m.joint(name).id];p=m.jnt_qposadr[m.joint(mm['joint']).id]
   d.qpos[a]=d.qpos[p]*float(mm.get('multiplier',1))+float(mm.get('offset',0))
  if command['reset']:reset();command['reset']=False
  mujoco.mj_forward(m,d);v.sync();time.sleep(1/30)
