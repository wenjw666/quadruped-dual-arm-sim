"""Measured free-fall, PD and contact diagnostics; no prescribed base motion."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
import mujoco
from qda_mujoco.dynamics import DynamicPlant
p=argparse.ArgumentParser();p.add_argument('--model',default='/ws/output/v2/dynamics.xml');a=p.parse_args()
plant=DynamicPlant(a.model);m,d=plant.model,plant.data
report={'model':a.model,'model_sha256':hashlib.sha256(Path(a.model).read_bytes()).hexdigest(),'nu':m.nu,'robot_mass_kg':float(m.body_subtreemass[plant.base_id]),'gravity_compensation_max':float(m.body_gravcomp.max())}
plant.reset(height=1.2);start=float(d.qpos[plant.root_q+2])
for _ in range(100):plant.step(enabled=False)
report['free_fall_drop_m']=start-float(d.qpos[plant.root_q+2])
assert report['free_fall_drop_m']>.1,'Root is constrained or gravity disabled'
plant.reset();report['initial_contacts']=[]
for c in d.contact:
 if c.dist<-.001:report['initial_contacts'].append({'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'distance':float(c.dist)})
minimum=10.;tilt=0.;contacts=[];self_overlap=0;start=time.monotonic()
for i in range(5000):
 plant.step()
 if not np.isfinite(d.qpos).all():raise AssertionError('Non-finite plant state')
 if i%50==0:
  s=plant.state();minimum=min(minimum,s['height']);tilt=max(tilt,float(np.arccos(np.clip(-s['gravity'][2],-1,1))));contacts.append(float(s['contacts'].sum()))
  self_overlap+=sum(c.dist<-.002 and m.geom_bodyid[c.geom1]!=0 and m.geom_bodyid[c.geom2]!=0 for c in d.contact)
report.update(pd_10s_height=float(plant.state()['height']),pd_min_height=minimum,pd_max_tilt_rad=tilt,mean_contact_feet=float(np.mean(contacts)),wall_s=time.monotonic()-start,
              effort_limit_ok=bool(np.all(np.abs(d.ctrl)<=plant.effort+1e-8)),finite_state=True,self_overlap_samples=int(self_overlap))
report['standing_gate_pass']=bool(minimum>.4 and tilt<.35 and report['effort_limit_ok'] and not report['initial_contacts'] and self_overlap==0)
out=Path(a.model).with_suffix('.audit.json');out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
