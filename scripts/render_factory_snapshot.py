"""Render the generated model at its actual shared home pose."""
import os
os.environ['MUJOCO_GL']='egl'
import json,math
from pathlib import Path
import mujoco,numpy as np
from PIL import Image
from qda_kinematics.kinematics import LEG_JOINTS,stance_command,AUBO_HOME_LEFT,AUBO_HOME_RIGHT
out=Path('/ws/output/expanded_demo')
m=mujoco.MjModel.from_xml_path('/ws/output/mujoco/factory.xml');d=mujoco.MjData(m)
values=dict(zip(LEG_JOINTS,stance_command()))
for side,pose in [('left',AUBO_HOME_LEFT),('right',AUBO_HOME_RIGHT)]:
    values.update({f'{side}_joint_link{i+1}':q for i,q in enumerate(pose)})
for n,q in values.items():d.qpos[m.jnt_qposadr[m.joint(n).id]]=q
mujoco.mj_forward(m,d);opt=mujoco.MjvOption();opt.geomgroup[3]=1
r=mujoco.Renderer(m,height=480,width=640)
for name,camera in [('factory_overview','overview'),('initial_head','head_camera')]:
    r.update_scene(d,camera=camera,scene_option=opt);Image.fromarray(r.render()).save(out/(name+'.png'))
    if camera=='head_camera':
        r.enable_depth_rendering();r.update_scene(d,camera=camera,scene_option=opt);np.save(out/'initial_depth.npy',r.render());r.disable_depth_rendering()
r.close()
print('FACTORY_RENDER_PASS',flush=True)
