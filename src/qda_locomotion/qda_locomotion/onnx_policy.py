"""Hash-checked transfer actor runtime; deployment acceptance remains explicit."""
import hashlib
import json
from pathlib import Path
import numpy as np
import onnxruntime as ort


class OnnxPolicy:
    def __init__(self,path,contract,*,experimental=False):
        path=Path(path)
        self.metadata=json.loads(path.with_suffix('.json').read_text())
        if not experimental and not self.metadata.get('runtime_ready'):
            raise ValueError('Policy has not passed task deployment acceptance')
        if self.metadata['contract'] != contract:
            raise ValueError('Policy/model contract mismatch')
        if hashlib.sha256(path.read_bytes()).hexdigest() != self.metadata['onnx_sha256']:
            raise ValueError('ONNX fingerprint mismatch')
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
        if self.session.get_inputs()[0].shape[-1] != 78 or self.session.get_outputs()[0].shape[-1] != 12:
            raise ValueError('Unexpected policy tensor dimensions')

    def predict(self,observation,deterministic=True):
        obs=np.asarray(observation,dtype=np.float32)
        if obs.shape != (78,) or not np.isfinite(obs).all():
            raise ValueError('Invalid policy observation')
        action=self.session.run(['action'],{'observation':obs[None,:]})[0]
        if action.shape != (1,12) or not np.isfinite(action).all():
            raise ValueError('Invalid policy output')
        return np.clip(action[0],-100.,100.),None
