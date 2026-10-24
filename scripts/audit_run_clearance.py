"""Offline 2-D navigation footprint audit against reference static boxes.

This is a geometric clearance diagnostic, not a contact-force measurement.
It excludes arm geometry and uses the configured padded Nav2 footprint.
"""
import csv,json,math
from pathlib import Path
import numpy as np

def polygon_gap(a,b):
    edges=np.concatenate((np.roll(a,-1,axis=0)-a,np.roll(b,-1,axis=0)-b))
    axes=np.column_stack((-edges[:,1],edges[:,0]));axes/=np.linalg.norm(axes,axis=1)[:,None]
    pa,pb=a@axes.T,b@axes.T
    separated=np.any((pa.max(0)<pb.min(0))|(pb.max(0)<pa.min(0)))
    if not separated:return 0.
    best=float('inf')
    for points,poly in ((a,b),(b,a)):
        v=np.roll(poly,-1,axis=0)-poly
        d=points[:,None,:]-poly[None,:,:]
        t=np.clip(np.sum(d*v[None,:,:],axis=-1)/np.sum(v*v,axis=-1),0,1)
        best=min(best,float(np.linalg.norm(d-t[:,:,None]*v[None,:,:],axis=-1).min()))
    return best

def audit(path):
    scene=json.loads(path.with_name('scene.json').read_text())
    boxes=[]
    for name,p,s,_ in scene['boxes']:
        x,y=p[:2];a,b=s[0]/2,s[1]/2
        boxes.append((name,np.array([[x+a,y+b],[x+a,y-b],[x-a,y-b],[x-a,y+b]])))
    minimum=float('inf');closest=None;overlap=[];count=0
    with path.with_name('trajectory.csv').open() as f:
        for row in csv.DictReader(f):
            x,y,yaw=[float(row[n]) for n in ('x','y','yaw')];c,s=math.cos(yaw),math.sin(yaw)
            poly=np.array([[.44,.57],[.44,-.57],[-.42,-.57],[-.42,.57]])@np.array([[c,s],[-s,c]])+[x,y]
            count+=1
            for name,box in boxes:
                gap=polygon_gap(poly,box)
                if gap<minimum:minimum,closest=gap,name
                if gap<1e-6:overlap.append(dict(sim_s=float(row['sim_s']),obstacle=name,x=x,y=y))
    result=dict(samples=count,min_reference_clearance_m=minimum if count else None,closest_obstacle=closest,
                footprint_overlap_samples=len(overlap),overlap_examples=overlap[:15],
                scope='Padded Nav2 footprint vs configured static boxes; not full-arm or force/contact validation')
    path.with_name('clearance_audit.json').write_text(json.dumps(result,indent=2))
    print(path.parent.name,'min gap',round(minimum,4),'overlaps',len(overlap),flush=True)
    return result

if __name__=='__main__':
    for path in sorted(Path('/ws/output/mujoco/runs').glob('*/summary.json')):
        audit(path)
