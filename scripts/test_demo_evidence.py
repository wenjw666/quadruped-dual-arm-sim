import os,sys,math
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
import cv2,mujoco,numpy as np
from qda_perception.detector_node import _depth_blob,_median_depth
sys.path.insert(0,'/ws/scripts')
from audit_run_clearance import polygon_gap
out=Path('/ws/output/expanded_demo')
rgb=cv2.imread(str(out/'initial_head.png'));depth=np.load(out/'initial_depth.npy')
hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
mask=cv2.bitwise_or(cv2.inRange(hsv,(0,90,50),(10,255,255)),cv2.inRange(hsv,(160,90,50),(180,255,255)))
m=mujoco.MjModel.from_xml_path('/ws/output/mujoco/factory.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d)
f=240/math.tan(math.radians(m.cam_fovy[m.camera('head_camera').id])/2)
blob=_depth_blob(mask,depth,f,.10);assert blob is not None,'Rendered red object must be visible'
u,v,_=blob;z=_median_depth(depth,u,v);assert math.isfinite(z)
point=np.array([(u-320)*z/f,(v-240)*z/f,z])
world=d.body('camera_optical_frame').xpos+d.body('camera_optical_frame').xmat.reshape(3,3)@point
error=np.linalg.norm(world-d.body('pick_object').xpos);assert error<.04,(world,error)
assert math.isnan(_median_depth(np.full((12,12),np.nan),6,6))
box=np.array([[0,0],[1,0],[1,1],[0,1]],float)
assert polygon_gap(box,box+.5)==0
assert abs(polygon_gap(box,box+[1.5,0])-.5)<1e-9
print('RGBD_RENDER_PROJECTION_PASS error_m=',error,'CLEARANCE_FIXTURES_PASS',flush=True)
