"""Measured task evidence, with explicit fallback and simulator boundaries."""
import csv
import json
import math
import threading
import time
import uuid
import hashlib
from datetime import datetime, timezone, timedelta
from pathlib import Path
from qda_kinematics.scene import scene_manifest, SCENARIO


class RunMetrics:
    def __init__(self, output, backend):
        self.lock = threading.RLock()
        self.output = Path(output)
        self.directory = self.output/'runs'/(datetime.now(timezone(timedelta(hours=9))).strftime('%Y%m%d_%H%M%S')+'_'+SCENARIO+'_'+uuid.uuid4().hex[:6])
        self.directory.mkdir(parents=True)
        self.started = time.monotonic()
        self.sim_start = None
        self.last = None
        self.rows = []
        self.data = dict(backend=backend, scenario=SCENARIO, status='RUNNING', path_m=0.,
                         yaw_travel_rad=0., planning_calls=0, planning_failures=0,
                         planning_wall_s=0., ik_collision_rejections=0, perception_fallbacks=[],
                         nav_legs=[], final_error_m=None, result_directory=str(self.directory),
                         limitations=['prescribed robot motion','virtual grasp','truth odometry for navigation'])
        (self.directory/'scene.json').write_text(json.dumps(scene_manifest(),indent=2))
        sources=[self.output/'qda_robot.urdf',self.output/'pick_place.world',
                 Path('/ws/src/qda_bringup/config/nav2_quad.yaml'),
                 Path('/ws/src/qda_kinematics/qda_kinematics/assembly.py'),
                 Path('/ws/src/qda_kinematics/qda_kinematics/scene.py'),
                 Path('/ws/src/qda_kinematics/qda_kinematics/kinematics.py'),
                 Path('/ws/src/qda_perception/qda_perception/detector_node.py'),
                 Path('/ws/src/qda_mujoco/qda_mujoco/model.py'),
                 Path('/ws/src/qda_mujoco/qda_mujoco/node.py'),
                 Path('/ws/src/qda_task/qda_task/pick_place_node.py')]
        snapshot=self.directory/'configuration';snapshot.mkdir()
        fingerprints={}
        for source in sources:
            if source.exists():
                content=source.read_bytes();fingerprints[str(source)]=hashlib.sha256(content).hexdigest()
                (snapshot/source.name).write_bytes(content)
        (self.directory/'source_sha256.json').write_text(json.dumps(fingerprints,indent=2))

    def odom(self, sim, x, y, yaw):
        with self.lock:
            if self.sim_start is None: self.sim_start=sim
            if self.last is not None:
                t,px,py,pa=self.last
                if sim-t<.1: return
                self.data['path_m']+=math.hypot(x-px,y-py)
                self.data['yaw_travel_rad']+=abs(math.atan2(math.sin(yaw-pa),math.cos(yaw-pa)))
            self.last=(sim,x,y,yaw)
            self.rows.append((sim,x,y,yaw))

    def event(self, text, sim):
        with self.lock:
            if '使用工位坐标' in text or '未看到绿色标记' in text:
                self.data['perception_fallbacks'].append(text)
            with (self.directory/'events.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(dict(sim_s=sim,wall_s=time.monotonic()-self.started,text=text),ensure_ascii=False)+'\n')
            if text.startswith('TASK_'): self.data['status']=text

    def snapshot(self):
        with self.lock:
            return dict(self.data,wall_s=time.monotonic()-self.started,
                        sim_s=0. if self.last is None else self.last[0]-self.sim_start)

    def finish(self, task_log):
        with self.lock:
            report=self.snapshot()
            if report['status']=='RUNNING':report['status']='INTERRUPTED'
            for path in (self.directory/'summary.json', self.output/'task_metrics.json'):
                path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            with (self.directory/'trajectory.csv').open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['sim_s','x','y','yaw']);w.writerows(self.rows)
            (self.directory/'task.log').write_bytes(Path(task_log).read_bytes())
