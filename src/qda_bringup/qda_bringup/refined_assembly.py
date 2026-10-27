"""Detailed central-column assembly and articulated vendor-CAD Robotiq 2F-85."""
from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET
import math
from ament_index_python.packages import get_package_share_directory
from qda_kinematics import assembly as A
from .arms import _origin, _paint, _inertial

WHITE='0.82 0.83 0.81 1'; SILVER='0.48 0.51 0.53 1'; DARK='0.055 0.065 0.075 1'; GLASS='0.015 0.027 0.035 1'
def fmt(values): return ' '.join(str(float(v)) for v in values)
def link(root,name): return root.find(f'link[@name="{name}"]')
def joint(root,name): return root.find(f'joint[@name="{name}"]')
def clear_geometry(element):
    for child in list(element):
        if child.tag in ('visual','collision'):element.remove(child)
def shape(l,kind,size,xyz=(0,0,0),rpy=(0,0,0),color=WHITE,collision=False):
    e=ET.SubElement(l,'collision' if collision else 'visual');_origin(e,fmt(xyz),fmt(rpy))
    attrs={'size':fmt(size)} if kind=='box' else ({'radius':str(size[0]),'length':str(size[1])} if kind=='cylinder' else {'radius':str(size[0])})
    ET.SubElement(ET.SubElement(e,'geometry'),kind,**attrs)
    if not collision:_paint(e,color)
def mesh(l,path,color=WHITE,xyz=(0,0,0),rpy=(0,0,0)):
    v=ET.SubElement(l,'visual');_origin(v,fmt(xyz),fmt(rpy));ET.SubElement(ET.SubElement(v,'geometry'),'mesh',filename='package://qda_description/meshes/'+path);_paint(v,color)
def set_joint(root,name,parent,xyz,rpy=(0,0,0)):
    j=joint(root,name);j.find('parent').set('link',parent)
    j.find('origin').set('xyz',fmt(xyz));j.find('origin').set('rpy',fmt(rpy))
def bolts(l,positions,radius=.003,axis=(0,0,0)):
    for xyz in positions:
        shape(l,'cylinder',(radius,.003),xyz,axis,SILVER)
        shape(l,'cylinder',(radius*.40,.0033),xyz,axis,DARK)
def add_fixed(root,name,parent,xyz):
    l=ET.SubElement(root,'link',name=name);_inertial(l)
    j=ET.SubElement(root,'joint',name=name+'_joint',type='fixed');_origin(j,fmt(xyz));ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name)
    return l

def robotiq(root,prefix):
    # Reuse the manufacturer's complete link/joint geometry and inertias.
    path=Path(get_package_share_directory('qda_description'))/'meshes/robotiq_2f85/source_macro.xacro'
    source=ET.parse(path).getroot().find('{*}macro')
    mapping={'robotiq_85_base_link':'gripper','robotiq_85_left_finger_tip_link':'finger_left','robotiq_85_right_finger_tip_link':'finger_right',
             'robotiq_85_left_knuckle_joint':'finger_left_joint','robotiq_85_right_knuckle_joint':'finger_right_joint','robotiq_85_base_joint':'gripper_joint'}
    def rename(s):
        s=s.replace('${prefix}','');return prefix+mapping.get(s,s)
    for original in source:
        if original.tag not in ('link','joint'):continue
        e=deepcopy(original)
        for child in e.iter():
            for attr in ('name','link','joint'):
                if child.get(attr) and '${prefix}' in child.get(attr):child.set(attr,rename(child.get(attr)))
            if child.tag=='mesh':child.set('filename',child.get('filename').replace('package://robotiq_description/meshes/','package://qda_description/meshes/robotiq_2f85/').replace('/2f_85/','/'))
        if e.tag=='link':
            for v in e.findall('visual'):
                filename=v.find('geometry/mesh').get('filename')
                _paint(v,DARK if 'base.dae' in filename or 'tip.dae' in filename else SILVER)
            for c in e.findall('collision'):
                for surface in c.findall('surface'):c.remove(surface)
        if e.get('name')==prefix+'gripper_joint':
            e.find('parent').set('link',prefix+'flange')
            for child in list(e):
                if child.tag.endswith('insert_block'):e.remove(child)
            _origin(e,fmt((0,0,A.GRIPPER_ADAPTER)),fmt((0,0,math.pi/2)))
        if e.get('name')==prefix+'finger_right_joint':
            # Keep the existing two-actuator ROS controller interface. Both
            # commanded values are positive close angles; the right axis flips.
            e.remove(e.find('mimic'));e.find('axis').set('xyz','0 1 0')
            e.find('limit').set('lower','0');e.find('limit').set('upper','.8')
        if e.get('name') in (prefix+'robotiq_85_right_finger_tip_joint',prefix+'robotiq_85_right_inner_knuckle_joint'):
            # The independently actuated right jaw must keep its own linkage
            # parallel under contact load, even when the left jaw deflects.
            e.find('mimic').set('joint',prefix+'finger_right_joint')
        root.append(e)
    flange=link(root,prefix+'flange')
    shape(flange,'cylinder',(.0315,.006),(0,0,.003),color=SILVER)
    bolts(flange,[(.024*math.cos(t),.024*math.sin(t),.0065) for t in [0,math.pi/2,math.pi,3*math.pi/2]],.0025)

def refine_assembly(root):
    deck=link(root,'arm_deck');clear_geometry(deck)
    set_joint(root,'arm_deck_joint','trunk',A.DECK_XYZ)
    mesh(deck,'refined/deck_shell.stl',DARK,xyz=(0,0,.018))
    shape(deck,'box',(.49,.35,.004),(0,0,.039),color=SILVER)
    shape(deck,'box',A.DECK_SIZE,(0,0,.018),collision=True)
    for y in (-.201,.201):
        for x in (-.18,-.135,-.09,-.045,0,.045,.09,.135,.18):shape(deck,'box',(.021,.002,.006),(x,y,.02),color='0.015 0.02 0.025 1')
    bolts(deck,[(x,y,.042) for x in (-.225,.225) for y in (-.155,.155)])
    mesh(deck,'refined/service_cable.stl',DARK)
    # Four short risers explain the mechanical load path to the original B2.
    for x in (-.18,.18):
        for y in (-.13,.13):shape(deck,'cylinder',(.014,.03),(x,y,-.015),color=DARK)
    mast=link(root,'sensor_mast');clear_geometry(mast)
    set_joint(root,'sensor_mast_joint','arm_deck',(A.MAST_XYZ[0]-A.DECK_XYZ[0],0,A.MAST_XYZ[2]-A.DECK_XYZ[2]))
    shape(mast,'cylinder',(.078,.012),(0,0,.006),color=SILVER)
    mesh(mast,'refined/column_shell.stl',WHITE)
    shape(mast,'cylinder',(.057,.262),(0,0,.14),collision=True)
    shape(mast,'cylinder',(.058,.006),(0,0,.220),color=SILVER)
    bolts(mast,[(.064*math.cos(t),.064*math.sin(t),.014) for t in [0,math.pi/2,math.pi,3*math.pi/2]],.003)
    shoulder=A.SHOULDER_Z-A.MAST_XYZ[2]
    shape(mast,'cylinder',(.060,.21),(0,0,shoulder),(math.pi/2,0,0),WHITE)
    shape(mast,'cylinder',(.056,.21),(0,0,shoulder),(math.pi/2,0,0),collision=True)
    # Rear service panel, cable gland and a neck above the shoulder bridge.
    shape(mast,'box',(.005,.06,.10),(-.056,0,.13),color=DARK)
    bolts(mast,[(-.060,y,z) for y in (-.022,.022) for z in (.095,.165)],.002,(0,math.pi/2,0))
    shape(mast,'box',(.041,.052,.124),(0,0,.368),color=SILVER)
    shape(mast,'box',(.036,.047,.120),(0,0,.368),collision=True)
    for z in (.335,.354,.373):
        shape(mast,'box',(.002,.040,.002),(0.021,0,z),color=DARK)
    for side,sign in [('left',1),('right',-1)]:
        pedestal=link(root,side+'_pedestal');clear_geometry(pedestal)
        shape(pedestal,'cylinder',(.059,.012),color=SILVER)
        shape(pedestal,'cylinder',(.058,.008),(0,0,-.007),color=DARK)
        shape(pedestal,'cylinder',(.056,.012),collision=True)
        rpy=A.LEFT_ARM_RPY if sign==1 else A.RIGHT_ARM_RPY
        set_joint(root,side+'_pedestal_joint','sensor_mast',(0,sign*.099,shoulder),rpy)
        set_joint(root,side+'_arm_mount_joint',side+'_pedestal',(0,0,.006))
        bolts(pedestal,[(.048*math.cos(t),.048*math.sin(t),.0065) for t in [0,math.pi/2,math.pi,3*math.pi/2]],.0025)
        for name in ('gripper','finger_left','finger_right'):
            old=link(root,side+'_'+name)
            if old is not None:root.remove(old)
        for name in ('gripper_joint','finger_left_joint','finger_right_joint'):
            old=joint(root,side+'_'+name)
            if old is not None:root.remove(old)
        robotiq(root,side+'_')
    camera=link(root,'camera_link');clear_geometry(camera)
    set_joint(root,'camera_joint','sensor_mast',tuple(A.CAMERA_XYZ[i]-A.MAST_XYZ[i] for i in range(3)),A.CAMERA_RPY)
    mesh(camera,'refined/camera_housing.stl',WHITE)
    mesh(camera,'refined/camera_glass.stl',GLASS)
    shape(camera,'box',(.042,.188,.044),collision=True)
    for y in (-.061,.061):
        shape(camera,'cylinder',(.010,.0015),(.0222,y,0),(0,math.pi/2,0),DARK)
        shape(camera,'cylinder',(.0065,.002),(.0232,y,0),(0,math.pi/2,0),'0.06 0.13 0.20 1')
        shape(camera,'cylinder',(.002,.0022),(.0234,y-.002,.002),(0,math.pi/2,0),'0.22 0.34 0.42 1')
    shape(camera,'cylinder',(.003,.0018),(.023,0,0),(0,math.pi/2,0),'0.08 0.13 0.15 1')
    shape(camera,'cylinder',(.0014,.0018),(.023,-.080,0),(0,math.pi/2,0),'0.12 0.75 0.35 1')
    # Move the retained Mid360 to the rear of the column; it no longer hides
    # the top first-person camera. Sensor origins remain explicit in URDF.
    set_joint(root,'lidar_joint','arm_deck',(A.LIDAR_XYZ[0]-A.DECK_XYZ[0],0,A.LIDAR_XYZ[2]-A.DECK_XYZ[2]),A.LIDAR_RPY)
    support=add_fixed(root,'rear_sensor_support','arm_deck',(-.16,0,.04))
    shape(support,'cylinder',(.012,.18),(0,0,.09),color=DARK)
    shape(support,'cylinder',(.022,.006),(0,0,.003),color=SILVER)
    # Explicit mimic state/command registration for Gazebo's ROS2 control.
    control=root.find('ros2_control')
    for j in root.findall('joint'):
        mimic=j.find('mimic')
        if mimic is None:continue
        entry=ET.SubElement(control,'joint',name=j.get('name'))
        ET.SubElement(entry,'param',name='mimic').text=mimic.get('joint')
        ET.SubElement(entry,'param',name='multiplier').text=mimic.get('multiplier','1')
        ET.SubElement(entry,'command_interface',name='position')
        ET.SubElement(entry,'state_interface',name='position')
        ET.SubElement(entry,'state_interface',name='velocity')
    for prefix in ('left','right'):
        from qda_kinematics.kinematics import AUBO_HOME_LEFT, AUBO_HOME_RIGHT
        for index,q in enumerate(AUBO_HOME_LEFT if prefix=='left' else AUBO_HOME_RIGHT):
            control.find(f'joint[@name="{prefix}_joint_link{index+1}"]/state_interface/param').text=str(q)
        for side in ('left','right'):
            control.find(f'joint[@name="{prefix}_finger_{side}_joint"]/state_interface/param').text='0'
    return root


def refine_srdf(srdf_xml, robot_xml):
    """Update home states and internal contacts of the rigid assembly/linkages."""
    root=ET.fromstring(srdf_xml);robot=ET.fromstring(robot_xml)
    from qda_kinematics.kinematics import AUBO_HOME_LEFT,AUBO_HOME_RIGHT
    for side,pose in [('left',AUBO_HOME_LEFT),('right',AUBO_HOME_RIGHT)]:
        for i,q in enumerate(pose):root.find(f'group_state[@name="home"][@group="{side}_arm"]/joint[@name="{side}_joint_link{i+1}"]').set('value',str(q))
    pairs={tuple(sorted((e.get('link1'),e.get('link2')))) for e in root.findall('disable_collisions')}
    def allow(a,b,reason):
        pair=tuple(sorted((a,b)))
        if pair not in pairs:ET.SubElement(root,'disable_collisions',link1=a,link2=b,reason=reason);pairs.add(pair)
    names=[x.get('name') for x in robot.findall('link')]
    parent={n:n for n in names}
    def find(n):
        while parent[n]!=n:n=parent[n]
        return n
    for j in robot.findall('joint'):
        a=j.find('parent').get('link');b=j.find('child').get('link')
        allow(a,b,'Adjacent')
        if j.get('type')=='fixed':parent[find(a)]=find(b)
    for i,a in enumerate(names):
        for b in names[i+1:]:
            if find(a)==find(b):allow(a,b,'Rigid assembly')
    for side in ('left','right'):
        mechanism=[n for n in names if n.startswith(side+'_robotiq_') or n in (side+'_gripper',side+'_finger_left',side+'_finger_right')]
        for i,a in enumerate(mechanism):
            for b in mechanism[i+1:]:allow(a,b,'Coupled gripper mechanism')
    return ET.tostring(root,encoding='unicode')
