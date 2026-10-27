"""Check a restored bundle in isolation, without reading original mesh files."""
import json
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
import onnxruntime as ort
from qda_mujoco.scene_control import robot_physics_signature

root=Path('/ws/output/release_v0.2.0/restore_test')
cache=Path('/ws/output/release_v0.2.0/restore_cache')
model=root/'output/v2/dynamics.xml';actor=root/'output/v2/runs/b2_transfer_v1/initial_policy.onnx'
metadata=json.loads(actor.with_suffix('.json').read_text());calibration=json.loads((root/'output/v2_audit/b2_command_response.json').read_text())
assert hashlib.sha256(model.read_bytes()).hexdigest()==metadata['contract']['model_sha256']==calibration['model_sha256']
assert hashlib.sha256(actor.read_bytes()).hexdigest()==metadata['onnx_sha256']
tree=ET.fromstring(model.read_text());assets=[]
for elem in tree.findall('.//asset/*'):
    name=elem.get('file')
    if not name:continue
    path=Path(name)
    if name.startswith('/tmp/qda_mujoco_meshes/'):
        path=cache/path.relative_to('/tmp/qda_mujoco_meshes')
    elif name.startswith('/ws/'):
        path=root/path.relative_to('/ws')
    elif not path.is_absolute():path=model.parent/path
    else:raise AssertionError('Unexpected model dependency: '+name)
    assert path.is_file();elem.set('file',str(path));assets.append(str(path))
restored=mujoco.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'))
signature=robot_physics_signature(restored,version=1) # v0.2.0 archive used the original signature format.
assert signature=='62998c467a2c0d05ace56274e4b7349a7e39dffbdaaeb1baa569a1158f161f98',signature
options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
session=ort.InferenceSession(str(actor),sess_options=options,providers=['CPUExecutionProvider'])
action=session.run(['action'],{'observation':np.zeros((1,78),dtype=np.float32)})[0]
assert action.shape==(1,12) and np.isfinite(action).all()
report=dict(passed=True,restored_meshes=len(assets),original_paths_used=False,robot_physics_signature=signature,
    onnx_output_shape=list(action.shape),model_sha256=metadata['contract']['model_sha256'],
    scope='Runtime assets restored into separate empty directories; existing container dependencies. Not a fresh-machine Docker build.')
Path('/ws/output/release_v0.2.0/restore_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
