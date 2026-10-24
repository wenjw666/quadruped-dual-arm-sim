import os
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
import json
from xml.etree import ElementTree as ET
import numpy as np
import mujoco,xacro
from PIL import Image,ImageDraw,ImageFont
from ament_index_python.packages import get_package_share_directory
from qda_bringup.arms import inject_aubo
from qda_mujoco.model import build_model
from qda_kinematics.kinematics import LEG_JOINTS,stance_command
out=Path('/ws/output/model_refinement_v1');out.mkdir(exist_ok=True)
urdf=inject_aubo(xacro.process_file(get_package_share_directory('qda_description')+'/urdf/qda_robot.urdf.xacro').toxml())
(out/'qda_refined.urdf').write_text(urdf)
(out/'studio.world').write_text('<sdf version="1.7"><world name="studio"><model name="overview_cam"><pose>2 -2 2 0 .6 2.3</pose></model></world></sdf>')
path=build_model(out/'qda_refined.urdf',out/'studio.world',out/'studio.xml')
root=ET.parse(path).getroot()
ET.SubElement(root.find('asset'),'texture',name='sky',type='skybox',builtin='gradient',rgb1='.78 .82 .87',rgb2='.93 .94 .96',width='256',height='256')
root.find('visual/headlight').set('ambient','.3 .3 .3');root.find('visual/headlight').set('diffuse','.6 .6 .6')
root.find('worldbody/light').set('castshadow','true')
root.find('worldbody/geom[@name="ground"]').set('rgba','.65 .68 .72 1')
root.find('visual/global').set('offwidth','1400');root.find('visual/global').set('offheight','1100')
ET.ElementTree(root).write(path)
m=mujoco.MjModel.from_xml_path(path);d=mujoco.MjData(m)
poses=json.loads((out/'poses.json').read_text())
values=dict(zip(LEG_JOINTS,stance_command()))
for side in ('left','right'):
 values.update({f'{side}_joint_link{i+1}':q for i,q in enumerate(poses[side])})
 values.update({f'{side}_finger_{finger}_joint':.15 for finger in ('left','right')})
manifest=json.loads((out/'studio.manifest.json').read_text())
for name,mm in manifest['mimics'].items():values[name]=values[mm['joint']]*float(mm.get('multiplier',1))+float(mm.get('offset',0))
for n,q in values.items():d.qpos[m.jnt_qposadr[m.joint(n).id]]=q
mujoco.mj_forward(m,d)
(out/'review_joints.json').write_text(json.dumps(values,indent=2))
np.save(out/'review_qpos.npy',d.qpos)
opt=mujoco.MjvOption();opt.geomgroup[3]=1
r=mujoco.Renderer(m,height=1100,width=1400)
views=[('perspective',145,-18,2.65,[0,0,.76]),('front',180,-7,2.6,[0,0,.76]),('side',90,-8,2.6,[0,0,.76]),('upper_detail',155,-12,1.45,[.10,0,.98]),('gripper_detail',145,-12,.62,[.25,.47,.72])]
for name,az,el,dist,look in views:
 c=mujoco.MjvCamera();c.azimuth=az;c.elevation=el;c.distance=dist;c.lookat[:]=look
 r.update_scene(d,camera=c,scene_option=opt)
 Image.fromarray(r.render()).save(out/f'{name}.png')
r.close()
print('RENDER_OK',m.nbody,m.njnt,flush=True)
# Save standalone SDF as a second parser/renderer target.
import subprocess
res=subprocess.run(['gz','sdf','-p',str(out/'qda_refined.urdf')],capture_output=True,text=True,timeout=90)
(out/'qda_refined.sdf').write_text(res.stdout)
(out/'sdf_check.log').write_text(res.stderr)
print('SDF conversion',res.returncode,flush=True)

