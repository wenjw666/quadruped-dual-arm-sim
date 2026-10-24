"""Build MJCF from the *same* URDF and generated SDF used by Gazebo.

No separately maintained coordinates: all robot transforms and world primitives
are imported. Robot collision meshes are convexified by MuJoCo; materials are
approximate. The default backend prescribes the robot motion, like Gazebo v1.
"""
import hashlib
import math
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory


def numbers(text, default='0 0 0'):
    return np.fromstring(text if text is not None else default, sep=' ')

def fmt(values):
    return ' '.join(f'{float(v):.10g}' for v in values)

def quat(rpy):
    xyzw = Rotation.from_euler('xyz', rpy).as_quat()
    return fmt(xyzw[[3, 0, 1, 2]])

def origin(element):
    if element is None:
        return {'pos':'0 0 0','quat':'1 0 0 0'}
    return {'pos':element.get('xyz','0 0 0'), 'quat':quat(numbers(element.get('rpy')))}

COLORS = {'Wood':(.59,.33,.13,1),'Grey':(.45,.48,.52,1),'DarkGrey':(.22,.25,.29,1),
          'Bricks':(.55,.24,.15,1),'CeilingTiled':(.75,.77,.79,1),'Yellow':(1,.8,0,1),
          'FlatBlack':(.08,.09,.11,1),'Blue':(.05,.20,.85,1),'Orange':(1,.45,.04,1),
          'ZincYellow':(.85,.80,.03,1),'Red':(.95,.03,.02,1),'Green':(.02,.95,.08,1)}


def build_model(urdf_path, sdf_path, output_path):
    import trimesh
    urdf, sdf = ET.parse(urdf_path).getroot(), ET.parse(sdf_path).getroot()
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    cache = Path('/tmp/qda_mujoco_meshes')
    cache.mkdir(exist_ok=True)
    root = ET.Element('mujoco', model='qda_factory')
    ET.SubElement(root,'compiler',angle='radian',autolimits='true',balanceinertia='true',fusestatic='false')
    ET.SubElement(root,'option',timestep='0.002',gravity='0 0 -9.81',integrator='implicitfast',iterations='40')
    visual = ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='640',offheight='480')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6',specular='.1 .1 .1')
    ET.SubElement(visual,'map',znear='.01',zfar='30')
    asset = ET.SubElement(root,'asset')
    ET.SubElement(asset,'texture',name='floor_texture',type='2d',builtin='checker',rgb1='.72 .74 .76',rgb2='.6 .63 .66',width='512',height='512')
    ET.SubElement(asset,'material',name='floor_material',texture='floor_texture',texrepeat='12 12',reflectance='.05')
    world = ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='1 -1 5',dir='0 0 -1',directional='true',castshadow='false')
    ET.SubElement(world,'geom',name='ground',type='plane',size='12 12 .1',material='floor_material',group='0',contype='1',conaffinity='3')
    meshes = {}
    def mesh_file(uri):
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/',1)
            source = Path(get_package_share_directory(package))/relative
        else:
            source = Path(uri)
        key = hashlib.sha1(('meters-v2'+str(source)+str(source.stat().st_mtime_ns)).encode()).hexdigest()[:16]
        dest = cache/(key+'.obj')
        if not dest.exists():
            scene = trimesh.load(str(source), force='scene')
            mesh = scene.to_geometry()
            # trimesh's COLLADA loader preserves raw coordinates but does not
            # apply <asset><unit>. Aubo visuals are millimetres; B2 is metres.
            if source.suffix.lower() == '.dae':
                unit = ET.parse(source).getroot().find('{*}asset/{*}unit')
                mesh.apply_scale(float(unit.get('meter', '1')) if unit is not None else 1.)
            mesh.export(dest)
        return str(dest)
    def urdf_geom(parent, entry, name, collision):
        geometry = entry.find('geometry')
        attrs = dict(name=name, **origin(entry.find('origin')))
        attrs.update(group='1' if collision else '2', contype='2' if collision else '0', conaffinity='1' if collision else '0')
        attrs['rgba'] = '0 0 0 0' if collision else '.82 .84 .87 1'
        if not collision:
            color = entry.find('material/color')
            if color is not None: attrs['rgba']=color.get('rgba')
        geom = geometry[0]
        if geom.tag == 'mesh':
            path = mesh_file(geom.get('filename'))
            key = (path, geom.get('scale','1 1 1'))
            if key not in meshes:
                meshes[key] = 'mesh_'+str(len(meshes))
                ET.SubElement(asset,'mesh',name=meshes[key],file=path,scale=key[1])
            attrs.update(type='mesh',mesh=meshes[key])
        elif geom.tag == 'box': attrs.update(type='box',size=fmt(numbers(geom.get('size'))/2))
        elif geom.tag == 'cylinder': attrs.update(type='cylinder',size=f"{geom.get('radius')} {float(geom.get('length'))/2}")
        elif geom.tag == 'sphere': attrs.update(type='sphere',size=geom.get('radius'))
        else: raise ValueError(f'Unsupported URDF geometry {geom.tag}')
        ET.SubElement(parent,'geom',**attrs)
    links = {l.get('name'):l for l in urdf.findall('link')}
    children = {}
    joints = [j for j in urdf.findall('joint') if j.find('parent') is not None]
    for j in joints: children.setdefault(j.find('parent').get('link'),[]).append(j)
    joint_names = []
    def body(link_name, parent, joint=None):
        attrs = dict(name=link_name, gravcomp='1')
        if joint is not None: attrs.update(origin(joint.find('origin')))
        else: attrs['pos']='0 0 .56'
        b = ET.SubElement(parent,'body',**attrs)
        if joint is None: ET.SubElement(b,'freejoint',name='robot_root')
        elif joint.get('type') not in ('fixed',):
            limit=joint.find('limit')
            ET.SubElement(b,'joint',name=joint.get('name'),type='slide' if joint.get('type')=='prismatic' else 'hinge',
                          axis=joint.find('axis').get('xyz'),range=f"{limit.get('lower')} {limit.get('upper')}",damping='1',armature='.01')
            joint_names.append(joint.get('name'))
        l=links[link_name]
        inertial=l.find('inertial')
        if inertial is not None:
            i=inertial.find('inertia')
            attrs=origin(inertial.find('origin'))
            # Rotate inertia into link axes; fullinertia and quat cannot coexist.
            iq = np.fromstring(attrs.pop('quat'),sep=' ')
            r = Rotation.from_quat(iq[[1,2,3,0]]).as_matrix()
            mat=np.array([[float(i.get('ixx')),float(i.get('ixy')),float(i.get('ixz'))],
                          [float(i.get('ixy')),float(i.get('iyy')),float(i.get('iyz'))],
                          [float(i.get('ixz')),float(i.get('iyz')),float(i.get('izz'))]])
            mat=r@mat@r.T
            ET.SubElement(b,'inertial',mass=inertial.find('mass').get('value'),
                          fullinertia=fmt([mat[0,0],mat[1,1],mat[2,2],mat[0,1],mat[0,2],mat[1,2]]),**attrs)
        else:
            ET.SubElement(b,'inertial',mass='.001',pos='0 0 0',diaginertia='1e-6 1e-6 1e-6')
        for kind in ('visual','collision'):
            for n,e in enumerate(l.findall(kind)): urdf_geom(b,e,f'{link_name}_{kind}_{n}',kind=='collision')
        if link_name=='camera_optical_frame':
            # MuJoCo cameras look down -Z, ROS optical cameras down +Z.
            hfov=float(urdf.findtext('.//sensor[@name="head_camera"]/camera/horizontal_fov'))
            fovy=math.degrees(2*math.atan(.75*math.tan(hfov/2)))
            ET.SubElement(b,'camera',name='head_camera',quat='0 1 0 0',fovy=str(fovy))
        for child in children.get(link_name,[]): body(child.find('child').get('link'),b,child)
    body('base_link',world)
    manifest=[]
    for model in sdf.findall('world/model'):
        name=model.get('name')
        if name in ('spawn_anchor','overview_cam'): continue
        p=numbers(model.findtext('pose'),'0 0 0 0 0 0')
        b=ET.SubElement(world,'body',name=name,pos=fmt(p[:3]),quat=quat(p[3:]))
        if name=='pick_object': ET.SubElement(b,'freejoint',name='object_free')
        link=model.find('link')
        for kind in ('visual','collision'):
            for n,entry in enumerate(link.findall(kind)):
                p=numbers(entry.findtext('pose'),'0 0 0 0 0 0')
                attrs=dict(name=f'{name}_{kind}_{n}',pos=fmt(p[:3]),quat=quat(p[3:]),group='3' if kind=='visual' else '0',
                           contype='0' if kind=='visual' else '1',conaffinity='0' if kind=='visual' else '3',
                           friction='.9 .005 .0001',rgba='0 0 0 0' if kind=='collision' else '.5 .5 .5 1')
                if kind=='visual':
                    material=entry.findtext('material/script/name','Gazebo/Grey').split('/')[-1]
                    attrs['rgba']=fmt(COLORS.get(material,(.5,.5,.5,1)))
                    if name=='floor_pad':
                        attrs.pop('rgba')
                        attrs['material']='floor_material'
                g=entry.find('geometry')[0]
                if g.tag=='box': attrs.update(type='box',size=fmt(numbers(g.findtext('size'))/2))
                elif g.tag=='cylinder': attrs.update(type='cylinder',size=f"{g.findtext('radius')} {float(g.findtext('length'))/2}")
                else: raise ValueError(g.tag)
                if name=='pick_object': attrs['mass']='.04' if kind=='collision' else '0'
                ET.SubElement(b,'geom',**attrs)
        manifest.append(dict(name=name,pose=model.findtext('pose'),collision_count=len(link.findall('collision'))))
    # Camera pose is imported from the Gazebo overview sensor.
    p=numbers(sdf.findtext('world/model[@name="overview_cam"]/pose'))
    r=Rotation.from_euler('xyz',p[3:]).as_matrix() @ np.array([[0,0,-1],[-1,0,0],[0,1,0]])
    q=Rotation.from_matrix(r).as_quat()
    ET.SubElement(world,'camera',name='overview',pos=fmt(p[:3]),quat=fmt(q[[3,0,1,2]]),fovy='59.36')
    ET.indent(root)
    ET.ElementTree(root).write(output,encoding='unicode')
    import json
    mimics = {j.get('name'):dict(j.find('mimic').attrib) for j in joints if j.find('mimic') is not None}
    output.with_suffix('.manifest.json').write_text(json.dumps({'source_urdf':str(urdf_path),'source_world':str(sdf_path),'joints':joint_names,'mimics':mimics,'scene':manifest},indent=2))
    return str(output)
