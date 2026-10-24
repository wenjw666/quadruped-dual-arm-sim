import concurrent.futures,json,urllib.request
from pathlib import Path
commit='be8aa38abb2f84e85adb9d6862092b9dfc0124c5'
base=f'https://raw.githubusercontent.com/robotiq/robotiq-ros/{commit}/'
root=Path('/ws/src/qda_description/meshes/robotiq_2f85');root.mkdir(parents=True,exist_ok=True)
files=['robotiq_base','left_knuckle','right_knuckle','left_inner_knuckle','right_inner_knuckle','left_finger','right_finger','left_finger_tip','right_finger_tip']
jobs=[]
for kind,ext in [('visual','dae'),('collision','stl')]:
 for stem in files:jobs.append((f'grippers/robotiq_description/meshes/{kind}/2f_85/{stem}.{ext}',root/kind/f'{stem}.{ext}'))
jobs += [('LICENSE',root/'LICENSE'),('grippers/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro',root/'source_macro.xacro')]
def get(job):
 src,dst=job;dst.parent.mkdir(parents=True,exist_ok=True)
 with urllib.request.urlopen(base+src,timeout=90) as r:dst.write_bytes(r.read())
 return str(dst),dst.stat().st_size
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
 for result in pool.map(get,jobs):print(result,flush=True)
(root/'SOURCE.json').write_text(json.dumps({'repository':'https://github.com/robotiq/robotiq-ros','commit':commit,'license':'BSD-3-Clause','files':[x[0] for x in jobs]},indent=2))
