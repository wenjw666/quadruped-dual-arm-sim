import mujoco,numpy as np,json
m=mujoco.MjModel.from_xml_path('/ws/output/model_refinement_v1/studio.xml');d=mujoco.MjData(m)
for q in [0.,.42,.8]:
 for side in ('left','right'):
  for finger in ('left','right'):d.qpos[m.jnt_qposadr[m.joint(f'{side}_finger_{finger}_joint').id]]=q
 for name,mm in json.load(open('/ws/output/model_refinement_v1/studio.manifest.json'))['mimics'].items():
  d.qpos[m.jnt_qposadr[m.joint(name).id]]=d.qpos[m.jnt_qposadr[m.joint(mm['joint']).id]]*float(mm.get('multiplier',1))
 mujoco.mj_forward(m,d)
 b=d.body('left_gripper');r=b.xmat.reshape(3,3)
 for side in ('left','right'):
  gid=m.geom(f'left_finger_{side}_collision_0').id;mid=m.geom_dataid[gid]
  v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
  w=v@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
  local=(w-b.xpos)@r
  print(q,side,local.min(0),local.max(0),flush=True)
