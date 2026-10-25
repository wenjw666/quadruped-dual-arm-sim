"""Fetch the reviewed B2 policy and its exact contract; never follows main."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

REVISION = '376d42c9b128f963ab08579762d5a216a976ce39'
FILES = {
    'base.yaml': ('policy/b2/base.yaml', '144ccfa91383b2d9cdbca25645e80f70d4ce2ac3f66c73c78b7751e6fc818aa6'),
    'config.yaml': ('policy/b2/robot_lab/config.yaml', 'b8ddd49db8c55ff436a4671b3ddf68e8134a8c22ecb9973f18febf426f1accad'),
    'policy.pt': ('policy/b2/robot_lab/policy.pt', 'd4aeb84894acda40f64feadaaad25d2be2ed3b8593b695b148e28673dff21e44'),
    'LICENSE': ('LICENSE', '6c18491053290fe1bada9775674b2b14493c4f0fdeab41e013dd5d4c82461d57'),
}
OFFICIAL_REVISION = '1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d'
OFFICIAL_FILES = {
    'official_b2.xml': ('unitree_robots/b2/b2.xml', 'ed6b51d00fb8593dd1c0e2360d10e06c5b86e422ec4583e92add0343ad24c9d8'),
    'LICENSE.unitree': ('LICENSE', 'a5d73fc4aca9074e3e6fe0b1a0ba763cf9514b2249b7390ed20fe8d53630bf25'),
}


def fetch(destination):
    destination.mkdir(parents=True, exist_ok=True)
    entries = {}
    sources = [(name, relative, digest, 'fan-ziqi/rl_sar', REVISION) for name, (relative, digest) in FILES.items()]
    sources += [(name, relative, digest, 'unitreerobotics/unitree_mujoco', OFFICIAL_REVISION)
                for name, (relative, digest) in OFFICIAL_FILES.items()]
    for name, relative, expected, repo, revision in sources:
        url = f'https://raw.githubusercontent.com/{repo}/{revision}/{relative}'
        path = destination / name
        data = path.read_bytes() if path.exists() else urlopen(url, timeout=60).read()
        actual = hashlib.sha256(data).hexdigest()
        if actual != expected:
            raise ValueError(f'Checksum mismatch: {path}; refusing to load or overwrite')
        if not path.exists():
            path.write_bytes(data)
        entries[name] = dict(url=url, sha256=actual, bytes=len(data))
    manifest = dict(repository='fan-ziqi/rl_sar', revision=REVISION, license='Apache-2.0',
                    official_model_revision=OFFICIAL_REVISION, official_model_license='BSD-3-Clause',
                    files=entries, runtime_ready=False, model='bare B2, not dual-arm trained')
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/ws/output/v2/upstream_b2'))
    fetch(parser.parse_args().output)
