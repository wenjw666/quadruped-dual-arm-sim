"""Check shared world topology and reference approach/checkpoint clearance."""
import os,sys,subprocess,math
from xml.etree import ElementTree as ET
import numpy as np
if len(sys.argv)==1:
    for profile in ('auto','north','south','side','legacy'):
        subprocess.run([sys.executable,__file__,profile],env=dict(os.environ,QDA_SCENARIO=profile),check=True)
    print('ALL_SCENE_PROFILES_PASS');raise SystemExit
from qda_kinematics.scene import *
from qda_bringup.world_sdf import world_xml
root=ET.fromstring(world_xml(False));models=root.findall('world/model')
names=[m.get('name') for m in models];assert len(names)==len(set(names))
for name,pos,size,_ in planning_boxes():
    model=root.find(f'world/model[@name="{name}"]');assert model is not None,name
    np.testing.assert_allclose(np.fromstring(model.findtext('pose'),sep=' ')[:3],pos)
    np.testing.assert_allclose(np.fromstring(model.findtext('.//collision/geometry/box/size'),sep=' '),size)

def intersects(x,y,yaw,pos,size):
    c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s],[s,c]])
    robot=np.array([[.44,.57],[.44,-.57],[-.42,-.57],[-.42,.57]])@R.T+[x,y]
    a,b=size[0]/2,size[1]/2
    box=np.array([[a,b],[a,-b],[-a,-b],[-a,b]])+pos[:2]
    for axis in (R[:,0],R[:,1],np.array([1,0]),np.array([0,1])):
        p,q=robot@axis,box@axis
        if max(p)<min(q) or max(q)<min(p):return False
    return True

c,s=math.cos(PLACE_YAW),math.sin(PLACE_YAW)
end=(PLACE_MARKER_XYZ[0]-c*.64+s*.32,PLACE_MARKER_XYZ[1]-s*.64-c*.32,PLACE_YAW)
poses=[(0,0,0),(.14,-.16,0),*ROUTE_WAYPOINTS,end]
for pose in poses:
    for name,p,size,_ in planning_boxes():assert not intersects(*pose,np.array(p),size),(SCENARIO,pose,name)
assert BOUNDS[0]>-10 and BOUNDS[1]<10 and BOUNDS[2]>-10 and BOUNDS[3]<10
assert abs(PLACE_TABLE_XYZ[2]+PLACE_TABLE_SIZE[2]/2-TABLE_TOP_Z)<1e-9
print(SCENARIO,'SDF/parity/approach clearance PASS',len(models),'models',flush=True)
