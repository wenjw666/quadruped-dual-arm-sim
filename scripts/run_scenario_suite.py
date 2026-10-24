"""Sequential real ROS/MuJoCo runs. Keeps every failure; never fabricates results."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
from datetime import datetime
p=argparse.ArgumentParser();p.add_argument('--scenarios',nargs='+',default=['south','north','side','auto'],choices=['auto','north','south','side','legacy']);p.add_argument('--repeat',type=int,default=1);p.add_argument('--timeout',type=float,default=1500)
a=p.parse_args()
if a.repeat<1:raise SystemExit('repeat must be positive')
if subprocess.run(['python3','/ws/scripts/stop_sim.py','--backend','mujoco','--check'],stdout=subprocess.DEVNULL).returncode==0:raise SystemExit('Stop the active MuJoCo run first; suite requires its own ROS domain 8.')
out=Path('/ws/output/expanded_demo')/('suite_'+datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
results=[]
for profile in a.scenarios:
 for repeat in range(a.repeat):
  tag=f'{profile}_{repeat+1}';started=time.time();summary=None
  env=dict(os.environ,ROS_DOMAIN_ID='8',QDA_SCENARIO=profile)
  with (out/(tag+'.log')).open('w') as log:
   proc=subprocess.Popen(['ros2','launch','qda_bringup','pick_place.launch.py',f'scenario:={profile}','backend:=mujoco','gui:=false','rviz:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   try:
    while time.time()-started<a.timeout and proc.poll() is None:
     path=Path('/ws/output/mujoco/task_metrics.json')
     if path.exists() and path.stat().st_mtime>started:
      try:summary=json.loads(path.read_text());break
      except json.JSONDecodeError:pass
     time.sleep(1)
   finally:
    if proc.poll() is None:
     os.killpg(proc.pid,signal.SIGINT)
     try:proc.wait(timeout=25)
     except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
  row=summary or dict(scenario=profile,status='STARTUP_FAILURE_OR_TIMEOUT',session_log=str(out/(tag+'.log')))
  if summary:
   from audit_run_clearance import audit
   clearance=audit(Path(summary['result_directory'])/'summary.json')
   row['footprint_overlap_samples']=clearance['footprint_overlap_samples']
   row['acceptance']='PASS' if row['status']=='TASK_SUCCESS' and clearance['footprint_overlap_samples']==0 else 'FAIL'
  else:row['acceptance']='FAIL'
  results.append(row);(out/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
  print(tag,row['status'],'error',row.get('final_error_m'),'path',row.get('path_m'),flush=True)
  time.sleep(3)
print('SUITE_RESULTS',out,flush=True)
raise SystemExit(0 if all(r['acceptance']=='PASS' for r in results) else 1)
