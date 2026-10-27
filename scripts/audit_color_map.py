"""Measure observed surface coverage; never synthesize points in missing regions."""
import argparse,json
from pathlib import Path
import numpy as np
from qda_kinematics.scene import BOUNDS,planning_boxes

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',default='/ws/output/mapping');a=p.parse_args();root=Path(a.root)
    xyz=np.load(root/'colored_map.npz')['xyz'];x0,x1,y0,y1=BOUNDS;cell=.1;walls={}
    for name,axis,value,spanaxis,low,high in [('west',0,x0+.075,1,y0,y1),('east',0,x1-.075,1,y0,y1),('south',1,y0+.075,0,x0,x1),('north',1,y1-.075,0,x0,x1)]:
        pts=xyz[(abs(xyz[:,axis]-value)<.045)&(xyz[:,2]>.05)&(xyz[:,2]<2.4)]
        bins=np.unique(np.floor(pts[:,[spanaxis,2]]/cell).astype(int),axis=0)
        walls[name]=dict(points=len(pts),observed_area_m2=round(len(bins)*cell**2,2),nominal_area_m2=round((high-low)*2.4,2),horizontal_span_m=round(float(np.ptp(pts[:,spanaxis])),2) if len(pts) else 0)
    floor=xyz[(xyz[:,2]<.04)&(xyz[:,0]>x0)&(xyz[:,0]<x1)&(xyz[:,1]>y0)&(xyz[:,1]<y1)]
    area=len(np.unique(np.floor(floor[:,:2]/cell).astype(int),axis=0))*cell**2
    # Approximate visible top/side samples on each known static obstacle; this
    # is an audit against geometry, not a source for the reconstructed cloud.
    objects={}
    for name,center,size,_ in planning_boxes():
        if name.startswith('wall_'):continue
        offset=abs(xyz-np.array(center));half=np.array(size)/2
        mask=(offset<=half+.045).all(axis=1)&(abs(offset-half)<.045).any(axis=1)
        objects[name]=int(mask.sum())
    result=dict(wall_surfaces=walls,observed_floor_m2=round(area,2),room_floor_m2=round((x1-x0)*(y1-y0),2),static_object_samples=objects,
                all_four_walls_observed=all(v['points']>500 for v in walls.values()),all_static_obstacles_observed=all(v>20 for v in objects.values()),
                notes='Coverage uses 10 cm surface bins. Occluded faces and hidden floor remain missing; no completeness percentage is inferred from bounding extent.')
    (root/'coverage_report.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
