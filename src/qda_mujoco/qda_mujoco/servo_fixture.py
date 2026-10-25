"""Optional optical-only calibration boards for repeatable IBVS acceptance.

These boards are not grasp targets. They change neither robot mass nor contacts.
World placement uses the initial camera pose once; the controller sees pixels only.
"""
from xml.etree import ElementTree as ET
import cv2
import numpy as np
import mujoco
from qda_kinematics.kinematics import AUBO_SERVO_LEFT,AUBO_SERVO_RIGHT
from scipy.spatial.transform import Rotation
from .dynamics import DynamicPlant


def add_servo_fixture(path):
    plant=DynamicPlant(path);root=ET.parse(path).getroot();world=root.find('worldbody')
    for index in plant.arm_indices:
        joint=plant.names[index];j=int(joint.rsplit('link',1)[1])-1
        plant.home[index]=(AUBO_SERVO_LEFT if joint.startswith('left') else AUBO_SERVO_RIGHT)[j]
    plant.reset()
    for _ in range(1500):plant.step()
    mujoco.mj_forward(plant.model,plant.data)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pattern=np.pad(cv2.aruco.drawMarker(dictionary,17,6),1,constant_values=255)
    cell=.08/6
    for side in ('left','right'):
        cid=plant.model.camera(side+'_wrist').id
        # MuJoCo camera x right, y up, -z forward -> optical convention.
        rotation=plant.data.cam_xmat[cid].reshape(3,3)@np.diag([1.,-1.,-1.])
        center=plant.data.cam_xpos[cid]+rotation@np.array([.012,-.008,.32])
        q=Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]]
        body=ET.SubElement(world,'body',name=side+'_servo_fixture',pos=' '.join(map(str,center)),quat=' '.join(map(str,q)))
        for row in range(8):
            for col in range(8):
                color='1 1 1 1' if pattern[row,col] else '0 0 0 1'
                ET.SubElement(body,'geom',type='box',name=f'{side}_marker_{row}_{col}',
                    pos=f'{(col-3.5)*cell} {(row-3.5)*cell} 0',size=f'{cell/2} {cell/2} .0001',
                    rgba=color,contype='0',conaffinity='0',group='2',mass='0')
    ET.ElementTree(root).write(path,encoding='unicode')
