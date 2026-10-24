import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from qda_kinematics.kinematics import _aubo_fk,_LEFT_MOUNT,_RIGHT_MOUNT,_AUBO_LO,_AUBO_HI
from qda_kinematics.assembly import LEFT_ARM_BASE,RIGHT_ARM_BASE
rng=np.random.default_rng(43)
results={}
for name,mount,base,target in [('left',_LEFT_MOUNT,LEFT_ARM_BASE,[.25,.47,.04]),('right',_RIGHT_MOUNT,RIGHT_ARM_BASE,[.25,-.47,.04]),('carry',_LEFT_MOUNT,LEFT_ARM_BASE,[.29,.32,.28])]:
 target=np.array(target);sign=1 if target[1]>0 else -1
 low=_AUBO_LO.copy();low[5]=.2 if name!="carry" else low[5]
 def error(q):
  pad,tool,pts=_aubo_fk(q[None,:],mount,np.array(base));p=pts[0]
  radial=np.linalg.norm(p[:,:2]-[.10,0],axis=1)
  inside=(p[:,2]>.13)&(p[:,2]<.56)
  return np.r_[6*(pad[0]-target),.7*(tool[0]-[0,0,-1]),np.maximum(.15-sign*p[2:,1],0)*2,np.maximum(.10-radial,0)*inside*10,np.maximum(.16-p[:-2,2],0)*3, max(0,p[2,2]-.38)*10 if name!="carry" else 0, .6*(p[2]-[-.02,sign*.44,.34]) if name!="carry" else np.zeros(3)]
 best=None
 for i in range(60):
  q0=rng.uniform(low,_AUBO_HI)
  fit=least_squares(error,q0,bounds=(low,_AUBO_HI),max_nfev=220)
  score=np.linalg.norm(error(fit.x))+.0003*np.linalg.norm(fit.x)
  if best is None or score<best[0]:best=(score,fit.x)
  if score<.005:break
 q=best[1];p,t,s=_aubo_fk(q[None,:],mount,np.array(base));results[name]=q.tolist();print(name,'score',best[0],'q',q.tolist(),'pad',p.tolist(),'points',s.tolist(),flush=True)
Path('/ws/output/model_refinement_v1/poses.json').write_text(json.dumps(results,indent=2))



