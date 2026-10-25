"""V2 virtual optical extrinsics, shared by MJCF and ROS TF (metres/radians).

These are provisional simulated mounts, not measured hardware calibration.
Optical +Z is forward, +X right and +Y down. No added mass or collision bodies.
"""
import math
from scipy.spatial.transform import Rotation

AUX_CAMERAS={
    'c0':dict(parent='base_link',xyz=(.33,0.,.12),rpy=(-math.pi/2,0.,-math.pi/2),fovy=65.),
    'left_wrist':dict(parent='left_link7',xyz=(.04,0.,.045),rpy=(0.,0.,0.),fovy=65.),
    'right_wrist':dict(parent='right_link7',xyz=(.04,0.,.045),rpy=(0.,0.,0.),fovy=65.),
}


def optical_quaternion(config):
    return Rotation.from_euler('xyz',config['rpy']).as_quat() # ROS xyzw


def add_cameras(robot):
    from xml.etree import ElementTree as ET
    for name,config in AUX_CAMERAS.items():
        parent=next(b for b in robot.iter('body') if b.get('name')==config['parent'])
        rot=Rotation.from_euler('xyz',config['rpy'])*Rotation.from_euler('x',math.pi)
        q=rot.as_quat()
        ET.SubElement(parent,'camera',name=name,pos=' '.join(map(str,config['xyz'])),
                      quat=' '.join(map(str,q[[3,0,1,2]])),fovy=str(config['fovy']))
