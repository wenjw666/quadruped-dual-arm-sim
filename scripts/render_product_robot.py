"""Presentation renders of the real model and recorded poses; never task evidence.

The orbit is a static model showcase. The replay interpolates recorded joint/odom
poses and does not simulate new success. Original camera footage is kept separately.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import json
from pathlib import Path
import cv2
import mujoco
import numpy as np
import xml.etree.ElementTree as ET
from qda_mujoco.dynamics import DynamicPlant


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--events');a=p.parse_args()
    folder=Path(a.out);folder.mkdir(parents=True,exist_ok=True)
    mode='replay' if a.events else 'hero';seconds=24 if a.events else 12;fps=12
    source=Path('/ws/output/v2_scene/factory_dynamic.xml' if a.events else '/ws/output/v2/dynamics.xml')
    tree=ET.parse(source)
    for elem in tree.findall('.//asset/*'):
        if elem.get('file') and not Path(elem.get('file')).is_absolute():elem.set('file',str((source.parent/elem.get('file')).resolve()))
    ET.SubElement(tree.find('asset'),'texture',dict(name='presentation_sky',type='skybox',builtin='gradient',
        rgb1='.86 .90 .91',rgb2='.95 .95 .92',width='512',height='3072'))
    presentation=Path('/ws/output/product_demo')/(mode+'_presentation.xml');tree.write(presentation)
    plant=DynamicPlant(presentation)
    m,d=plant.model,plant.data;m.vis.global_.offwidth=1200;m.vis.global_.offheight=800
    renderer=mujoco.Renderer(m,height=800,width=1200)
    opt=mujoco.MjvOption();opt.geomgroup[1]=0;opt.geomgroup[2]=1
    camera=mujoco.MjvCamera();camera.distance=2.35;camera.elevation=-16
    target=folder/(mode+'.mp4');writer=cv2.VideoWriter(str(target),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1200,800))
    if not writer.isOpened():raise RuntimeError('Video writer unavailable')
    rows=[]
    if a.events:
        events=[json.loads(x) for x in Path(a.events).read_text().splitlines()]
        rows=[x for x in events if x['kind']=='joints' and x['data']['base'] and x['sim']>1.]
        # End at mission terminal, so later stationary recording doesn't dominate.
        terminal=next((x['wall'] for x in events if x['kind']=='mission' and x['data'].get('status')=='SUCCEEDED' and x['wall']>rows[0]['wall']),rows[-1]['wall'])
        rows=[x for x in rows if x['wall']<=terminal]
        times=np.array([x['wall'] for x in rows]);poses=[]
        for row in rows:
            q=d.qpos.copy();b=row['data']['base'];q[plant.root_q:plant.root_q+7]=b['position']+b['quaternion_wxyz']
            for name,value in row['data']['joints'].items():
                try:q[m.jnt_qposadr[m.joint(name).id]]=value
                except KeyError:pass
            poses.append(q)
        poses=np.array(poses)
    try:
        for f in range(seconds*fps):
            progress=f/max(1,seconds*fps-1)
            if rows:
                t=times[0]+progress*(times[-1]-times[0]);i=max(0,min(len(times)-2,np.searchsorted(times,t)-1))
                blend=(t-times[i])/max(1e-9,times[i+1]-times[i]);d.qpos[:]=poses[i]*(1-blend)+poses[i+1]*blend
                q=d.qpos[plant.root_q+3:plant.root_q+7];q/=np.linalg.norm(q)
                camera.azimuth=145;camera.distance=2.6;camera.elevation=-22
            else:camera.azimuth=125+80*progress
            mujoco.mj_forward(m,d);camera.lookat[:]=d.xpos[plant.base_id]+np.array([0,0,.20])
            renderer.update_scene(d,camera=camera,scene_option=opt)
            frame=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR);writer.write(frame)
            if f in (0,seconds*fps//2):cv2.imwrite(str(folder/f'{mode}-{f}.png'),frame)
    finally:writer.release();renderer.close()
    report=dict(kind='recorded pose presentation replay' if rows else 'static model orbit',seconds=seconds,fps=fps,
        source=a.events,physics_claim=False,source_wall_range=[float(times[0]),float(times[-1])] if rows else None)
    target.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()
