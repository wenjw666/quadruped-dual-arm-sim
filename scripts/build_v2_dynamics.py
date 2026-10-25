"""Build a separate V2 model without changing the V1 generated model."""
import argparse,json
from pathlib import Path
from qda_mujoco.model import build_model
from qda_mujoco.dynamics import build_dynamic_model
p=argparse.ArgumentParser()
p.add_argument('--source-dir',default='/ws/output/mujoco')
p.add_argument('--output',default='/ws/output/v2/dynamics.xml')
p.add_argument('--training',action='store_true')
a=p.parse_args();source=Path(a.source_dir);out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
# Rebuild mesh conversions if the container's /tmp cache was cleared.
shared=build_model(source/'qda_robot.urdf',source/'pick_place.world',out.with_name('shared_source.xml'))
print(json.dumps(build_dynamic_model(shared,source/'qda_robot.urdf','/ws/src/qda_moveit_config/config/qda_robot.srdf',out,a.training),indent=2))
