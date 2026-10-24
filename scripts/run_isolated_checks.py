"""Parallel functional checks only; wall times are not benchmark comparisons."""
import os,json,subprocess,signal,time,argparse
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from audit_run_clearance import audit
root=Path('/ws/output/expanded_demo')/('isolated_'+datetime.now().strftime('%Y%m%d_%H%M%S'));root.mkdir()
parser=argparse.ArgumentParser();parser.add_argument('--scenarios',nargs='+',choices=['auto','south','north','side'],default=['side','auto','south']);args=parser.parse_args()
def check(spec):
 profile,domain=spec;out=root/profile;out.mkdir();started=time.time();summary=None
 env=dict(os.environ,ROS_DOMAIN_ID=str(domain),QDA_SCENARIO=profile,LP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='2')
 with (out/'session.log').open('w') as log:
  proc=subprocess.Popen(['ros2','launch','qda_bringup','pick_place.launch.py',f'scenario:={profile}','backend:=mujoco',f'output_dir:={out}','gui:=false','rviz:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   while proc.poll() is None and time.time()-started<1800:
    result=out/'task_metrics.json'
    if result.exists():
     try:summary=json.loads(result.read_text());break
     except json.JSONDecodeError:pass
    time.sleep(1)
  finally:
   if proc.poll() is None:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=25)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
 if summary:
  run=Path(summary['result_directory'])
  (run/'validation_environment.json').write_text(json.dumps(dict(parallel_functional_check=True,ROS_DOMAIN_ID=domain,LP_NUM_THREADS=2,OPENBLAS_NUM_THREADS=1,OMP_NUM_THREADS=2),indent=2))
  clearance=audit(run/'summary.json')
  summary['acceptance']='PASS' if summary['status']=='TASK_SUCCESS' and clearance['footprint_overlap_samples']==0 else 'FAIL'
 else:summary=dict(scenario=profile,status='STARTUP_FAILURE_OR_TIMEOUT',acceptance='FAIL')
 (out/'acceptance.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
 print(profile,summary['status'],summary['acceptance'],flush=True)
 return summary
domains={'side':21,'auto':22,'south':23,'north':24}
with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(check,[(p,domains[p]) for p in args.scenarios]))
(root/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
print('ISOLATED_RESULTS',root,flush=True)
raise SystemExit(0 if all(r['acceptance']=='PASS' for r in results) else 1)
