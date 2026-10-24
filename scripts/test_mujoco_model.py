import os
os.environ.setdefault('MUJOCO_GL','egl')
import sys
sys.path.insert(0,'/ws/src/qda_mujoco')
from qda_mujoco.model import build_model
import mujoco
import numpy as np
from pathlib import Path
from qda_kinematics.kinematics import LEG_JOINTS, stance_command, AUBO_HOME_LEFT, AUBO_HOME_RIGHT
from qda_kinematics.assembly import LEFT_ARM_BASE, GRIPPER_OPEN
from PIL import Image

path=build_model('/ws/output/mujoco/qda_robot.urdf','/ws/output/mujoco/pick_place.world','/ws/output/expanded_demo/model_test.xml')
model=mujoco.MjModel.from_xml_path(path)
data=mujoco.MjData(model)
targets=dict(zip(LEG_JOINTS,stance_command()))
for prefix,home in [('left',AUBO_HOME_LEFT),('right',AUBO_HOME_RIGHT)]:
    targets.update({f'{prefix}_joint_link{i+1}':v for i,v in enumerate(home)})
    targets.update({f'{prefix}_finger_{side}_joint':GRIPPER_OPEN for side in ['left','right']})
for name,value in targets.items():data.qpos[model.jnt_qposadr[model.joint(name).id]]=value
mujoco.mj_forward(model,data)
print('Compiled:',model.nbody,'bodies',model.njnt,'joints',model.ngeom,'geoms',model.nmesh,'meshes',flush=True)
print(mujoco.mj_multiRay.__doc__,flush=True)
for name in ['base_link','camera_optical_frame','lidar_link','pick_object']:
    print(name,data.body(name).xpos,flush=True)
renderer=mujoco.Renderer(model,height=480,width=640)
option=mujoco.MjvOption();option.geomgroup[3]=1
for camera in ['head_camera','overview']:
    renderer.update_scene(data,camera=camera,scene_option=option)
    pixels=renderer.render().copy()
    if camera=='head_camera':rgb=pixels
    Image.fromarray(pixels).save(f'/ws/output/mujoco/{camera}_initial.png')
renderer.enable_depth_rendering();renderer.update_scene(data,camera='head_camera',scene_option=option)
depth=renderer.render();print('Depth finite fraction',np.mean(np.isfinite(depth)),'range',depth.min(),depth.max(),flush=True)
renderer.close()
assert len(targets)==30
assert np.all(np.isfinite(data.qpos))
import cv2
from qda_perception.detector_node import _depth_blob
hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
mask=cv2.bitwise_or(cv2.inRange(hsv,(0,90,50),(10,255,255)),cv2.inRange(hsv,(160,90,50),(180,255,255)))
f=240/np.tan(np.radians(model.cam_fovy[model.camera('head_camera').id])/2)
blob=_depth_blob(mask,depth,f,.10)
assert blob is not None,'Red target missing from rendered RGB camera'
u,v,area=blob
f=240/np.tan(np.radians(model.cam_fovy[model.camera('head_camera').id])/2)
point=np.array([(u-320)*depth[v,u]/f,(v-240)*depth[v,u]/f,depth[v,u]])
world=data.body('camera_optical_frame').xpos+data.body('camera_optical_frame').xmat.reshape(3,3)@point
error=np.linalg.norm(world-data.body('pick_object').xpos)
assert error<.05,(world,error)
directions=np.column_stack((np.cos(np.linspace(-np.pi,np.pi,360)),np.sin(np.linspace(-np.pi,np.pi,360)),np.zeros(360)))
ids=np.full(360,-1,np.int32);dist=np.full(360,-1.)
mujoco.mj_multiRay(model,data,data.body('nav_scan_link').xpos.copy(),directions.flatten(),np.array([1,0,0,1,0,0],np.uint8),1,-1,ids,dist,360,12.)
assert np.sum(dist>0)>250,'Lidar must hit surrounding walls/obstacles'
assert .2<np.min(dist[dist>0])<1.,'Expected nearby rack/table'
assert all(np.max(model.mesh_vert[model.mesh_vertadr[i]:model.mesh_vertadr[i]+model.mesh_vertnum[i]],axis=0).max()<2 for i in range(model.nmesh)),'Mesh unit conversion failed'
print('RGB-D target error',error,'m; scan hits',np.sum(dist>0),flush=True)
from qda_kinematics.kinematics import AUBO_CARRY_LEFT, _aubo_fk, _LEFT_MOUNT, GRASP_PAD
for pose in (AUBO_HOME_LEFT,AUBO_CARRY_LEFT):
    for i,q in enumerate(pose):data.qpos[model.jnt_qposadr[model.joint(f'left_joint_link{i+1}').id]]=q
    mujoco.mj_forward(model,data)
    actual=data.body('left_gripper').xpos+GRASP_PAD*data.body('left_gripper').xmat.reshape(3,3)[:,2]-data.body('base_link').xpos
    expected=_aubo_fk(np.array(pose)[None,:],_LEFT_MOUNT,np.array(LEFT_ARM_BASE))[0][0]
    np.testing.assert_allclose(actual,expected,atol=1e-7)
initial=data.qpos.copy();free=int(model.jnt_qposadr[model.joint('object_free').id])
for _ in range(500):
    data.qpos[:free]=initial[:free];data.qvel[:int(model.jnt_dofadr[model.joint('object_free').id])]=0
    mujoco.mj_step(model,data)
assert abs(data.qpos[free+2]-.57)<.005,data.qpos[free:free+3]
print('URDF/IK FK parity and released-cube table stability passed',flush=True)
print('MODEL_AND_RENDER_OK',flush=True)
