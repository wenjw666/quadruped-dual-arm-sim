"""Reference layout and measured trajectories: never confuse hints with plans."""
import json,os,glob,csv
os.environ.setdefault('MPLBACKEND','Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from pathlib import Path
out=Path('/ws/output/expanded_demo');out.mkdir(exist_ok=True)
scene=json.loads(Path('/ws/output/mujoco/scene_manifest.json').read_text())
fig,ax=plt.subplots(figsize=(11,9),facecolor='#101d2b');ax.set_facecolor('#16283a')
for name,p,size,slab in scene['boxes']:
    color='#e6b46d' if 'table' in name or 'station' in name else '#58738c'
    ax.add_patch(Rectangle((p[0]-size[0]/2,p[1]-size[1]/2),size[0],size[1],facecolor=color,edgecolor='#c7d8e5',alpha=.8))
    if not name.startswith('wall'):ax.text(p[0],p[1],name,ha='center',va='center',fontsize=8,color='white')
for name,points in scene['route_options'].items():
    xs,ys,_=zip(*points);color='#53bfff' if name=='north' else '#f5cc61'
    ax.plot(xs,ys,'o--',color=color,label=f'{name}: reference checkpoints')
ax.scatter([0],[0],s=140,marker='^',color='#7ef0be',label='Robot start')
ax.scatter([scene['pick'][0]],[scene['pick'][1]],s=70,color='#ff7c79',label='Pick')
ax.scatter([scene['place'][0]],[scene['place'][1]],s=90,marker='*',color='#72e9a3',label='Selected place')
latest={}
for f in sorted(Path('/ws/output').glob('**/runs/*/summary.json'),key=lambda p:p.parent.name):
    result=json.loads(f.read_text())
    if result.get('scenario') in ('auto','north','south','side'):latest[result['scenario']]=f
for f in latest.values():
    r=json.loads(f.read_text());path=f.with_name('trajectory.csv')
    if r.get('scenario') not in ('auto','north','south','side') or not path.exists():continue
    rows=list(csv.DictReader(path.open()))
    audit=f.with_name('clearance_audit.json')
    clearance=json.loads(audit.read_text()) if audit.exists() else {}
    accepted=r['status']=='TASK_SUCCESS' and clearance.get('footprint_overlap_samples')==0
    if rows:ax.plot([float(x['x']) for x in rows],[float(x['y']) for x in rows],lw=1.8,label=f"Measured {r['scenario']}: {'PASS' if accepted else 'NOT ACCEPTED'}")
x0,x1,y0,y1=scene['bounds'];ax.set(xlim=(x0-.25,x1+.25),ylim=(y0-.3,y1+.3),aspect='equal',xlabel='x [m]',ylabel='y [m]')
ax.set_title('EXPANDED FACTORY | shared Gazebo / MuJoCo layout\nDashed = route hints; solid = measured odometry',color='white',loc='left',pad=20)
ax.tick_params(colors='#c7d8e5');ax.xaxis.label.set_color('white');ax.yaxis.label.set_color('white')
ax.grid(alpha=.12);ax.legend(loc='lower left',fontsize=8,facecolor='#223b50',labelcolor='white')
fig.tight_layout();fig.savefig(out/'map_and_routes.png',dpi=160);plt.close(fig)
print(out/'map_and_routes.png')
