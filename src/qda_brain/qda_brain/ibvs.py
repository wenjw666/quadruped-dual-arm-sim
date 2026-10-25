"""Four-corner IBVS with normalized coordinates and measured PnP depth."""
import numpy as np


def command(corners,depths,k,desired_depth=.30,marker_size=.08,gain=.6):
    corners=np.asarray(corners,dtype=float).reshape(4,2);z=np.asarray(depths,dtype=float).reshape(4)
    if not np.all(np.isfinite(corners)) or not np.all(np.isfinite(z)) or np.any(z<.08) or np.any(z>1.5):raise ValueError('INVALID_DEPTH')
    if not .15<=desired_depth<=.8:raise ValueError('INVALID_DESIRED_DEPTH')
    fx,fy,cx,cy=k[0,0],k[1,1],k[0,2],k[1,2]
    xy=(corners-[cx,cy])/[fx,fy];a=marker_size/(2*desired_depth)
    desired=np.array([[-a,-a],[a,-a],[a,a],[-a,a]])
    rows=[]
    for (x,y),d in zip(xy,z):rows.extend([[-1/d,0,x/d,x*y,-(1+x*x),y],[0,-1/d,y/d,1+y*y,-x*y,-x]])
    L=np.asarray(rows);u,s,vt=np.linalg.svd(L,full_matrices=False)
    condition=s[0]/max(s[-1],1e-12)
    if condition>5000:raise ValueError('ILL_CONDITIONED_FEATURES')
    error=(xy-desired).reshape(-1)
    velocity=-gain*(vt.T @ ((s/(s*s+1e-5))*(u.T@error)))
    velocity[:3]*=min(1.,.025/max(np.linalg.norm(velocity[:3]),1e-12))
    velocity[3:]*=min(1.,.12/max(np.linalg.norm(velocity[3:]),1e-12))
    return velocity,float(np.sqrt(np.mean(((xy-desired)*[fx,fy])**2))),float(condition)
