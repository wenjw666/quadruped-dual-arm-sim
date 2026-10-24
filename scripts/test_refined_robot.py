import json,math,os
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
from qda_kinematics.kinematics import _aubo_fk,_LEFT_MOUNT,_RIGHT_MOUNT,AUBO_HOME_LEFT,AUBO_HOME_RIGHT,AUBO_CARRY_LEFT,left_grasp_joints,GRASP_PAD
from qda_kinematics.assembly import LEFT_ARM_BASE,RIGHT_ARM_BASE
import mujoco
out=Path('/ws/output/model_refinement_v1')
m=mujoco.MjModel.from_xml_path(str(out/'studio.xml'));d=mujoco.MjData(m)
for prefix,home,mount,base in [('left',AUBO_HOME_LEFT,_LEFT_MOUNT,LEFT_ARM_BASE),('right',AUBO_HOME_RIGHT,_RIGHT_MOUNT,RIGHT_ARM_BASE)]:
 for i,q in enumerate(home):d.qpos[m.jnt_qposadr[m.joint(f'{prefix}_joint_link{i+1}').id]]=q
 mujoco.mj_forward(m,d)
 actual=d.body(prefix+'_gripper').xpos+GRASP_PAD*d.body(prefix+'_gripper').xmat.reshape(3,3)[:,2]-d.body('base_link').xpos
 predicted=_aubo_fk(np.array(home)[None,:],mount,np.array(base))[0][0]
 np.testing.assert_allclose(actual,predicted,atol=1e-7)
 print(prefix,'FK agreement',np.max(np.abs(actual-predicted)),flush=True)
robot=ET.parse(out/'qda_refined.urdf').getroot()
names=[x.get('name') for x in robot.findall('link')];assert len(set(names))==len(names)
assert len(robot.findall('joint'))==len(names)-1
for j in robot.findall('joint'):
 assert j.find('parent').get('link') in names and j.find('child').get('link') in names
for target in [[.64,.32,.01],[.64,.32,.13],[.69,.32,.01]]:
 solutions=left_grasp_joints(target,count=3)
 print('grasp',target,'solutions',len(solutions),flush=True)
 assert solutions,'No reachable grasp family for new shoulder placement'
 for q in solutions:
  pad,tool,_=_aubo_fk(np.array(q)[None,:],_LEFT_MOUNT,np.array(LEFT_ARM_BASE));assert np.linalg.norm(pad[0]-target)<.03
print('STRUCTURE_FK_IK_PASS',flush=True)
