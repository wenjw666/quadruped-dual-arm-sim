"""Fail early with an actionable error before starting the V2 ROS graph."""
import hashlib
import importlib.util
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

base=Path('/ws/output/v2')
model=base/'dynamics.xml'
actor=base/'runs/b2_transfer_v1/initial_policy.onnx'
response=Path('/ws/output/v2_audit/b2_command_response.json')
required=[model,actor,actor.with_suffix('.json'),response,base/'upstream_b2/base.yaml',base/'upstream_b2/config.yaml']
missing=[str(p) for p in required if not p.is_file()]
missing.extend('Python dependency: '+name for name in ('mujoco','onnxruntime','gymnasium','coacd') if importlib.util.find_spec(name) is None)
if missing:raise SystemExit('V2 assets are not ready. See docs/V2_FACTORY_INTEGRATION.md. Missing:\n'+'\n'.join(missing))
for element in ET.parse(model).findall('.//asset/*'):
    filename=element.get('file')
    if filename:
        path=Path(filename)
        if not path.is_absolute():path=model.parent/path
        if not path.is_file():missing.append(str(path))
if missing:raise SystemExit('V2 mesh cache is incomplete. Restore the verified release runtime bundle:\n'+'\n'.join(missing))
meta=json.loads(actor.with_suffix('.json').read_text());calibration=json.loads(response.read_text())
digest=hashlib.sha256(model.read_bytes()).hexdigest()
if meta['contract']['model_sha256']!=digest or calibration['model_sha256']!=digest:
    raise SystemExit('Reference model changed; re-export and revalidate its policy/calibration before launching.')
if hashlib.sha256(actor.read_bytes()).hexdigest()!=meta['onnx_sha256'] or not meta.get('reference_equivalent'):
    raise SystemExit('The calibrated scene requires the verified original-equivalent ONNX actor.')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/qda_locomotion'))
from qda_locomotion.upstream_b2 import EXPECTED
for name in ('base.yaml','config.yaml'):
    if hashlib.sha256((base/'upstream_b2'/name).read_bytes()).hexdigest()!=EXPECTED[name]:
        raise SystemExit('B2 upstream configuration checksum mismatch: '+name)
print('V2 experimental scene assets verified; task deployment acceptance remains false.')
