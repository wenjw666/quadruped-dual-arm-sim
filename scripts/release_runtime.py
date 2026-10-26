"""Package/install the exact calibrated V2 runtime without rewriting model hashes.

Linux container paths are intentionally fixed at /ws and /tmp. No blind tar
extraction: validate all hashes, destinations and existing contents before writes.
The archive is a private engineering snapshot, not a redistributable robot SDK.
"""
import argparse
import hashlib
import io
import json
import tarfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath


FILES = [
    'output/v2/dynamics.xml', 'output/v2/dynamics.dynamic.json',
    'output/v2/upstream_b2/base.yaml', 'output/v2/upstream_b2/config.yaml',
    'output/v2/upstream_b2/LICENSE', 'output/v2/upstream_b2/LICENSE.unitree',
    'output/v2/upstream_b2/manifest.json',
    'output/v2/runs/b2_transfer_v1/initial_policy.onnx',
    'output/v2/runs/b2_transfer_v1/initial_policy.json',
    'output/v2_audit/b2_command_response.json',
]


def digest(data):return hashlib.sha256(data).hexdigest()


def destination(name, workspace, cache):
    p=PurePosixPath(name)
    if p.is_absolute() or '..' in p.parts or '\\' in name:raise ValueError('Unsafe archive path: '+name)
    if name.startswith('workspace/output/v2/') or name=='workspace/output/v2_audit/b2_command_response.json':
        root=workspace;relative=p.relative_to('workspace')
    elif name.startswith('cache/'):
        root=cache;relative=p.relative_to('cache')
    else:raise ValueError('Unexpected destination: '+name)
    result=root.joinpath(*relative.parts)
    if not result.resolve().is_relative_to(root.resolve()):raise ValueError('Destination escapes root')
    return result


def pack(archive,workspace,cache):
    if archive.exists():raise FileExistsError(archive)
    sources={'workspace/'+n:workspace/n for n in FILES}
    tree=ET.parse(workspace/'output/v2/dynamics.xml')
    for elem in tree.findall('.//asset/*'):
        if not elem.get('file'):continue
        path=Path(elem.get('file'))
        if not path.is_absolute():path=(workspace/'output/v2'/path).resolve()
        if path.is_relative_to(cache):name='cache/'+path.relative_to(cache).as_posix()
        elif path.is_relative_to(workspace/'output/v2'):name='workspace/'+path.relative_to(workspace).as_posix()
        else:raise ValueError('Unknown model asset: '+str(path))
        sources[name]=path
    manifest=dict(schema=1,version='v0.2.0',runtime_ready=False,workspace='/ws',cache='/tmp/qda_mujoco_meshes',
        purpose='Exact calibrated simulation baseline; experimental RL; no physical grasp acceptance',files={})
    for name,path in sorted(sources.items()):
        data=path.read_bytes();manifest['files'][name]=dict(bytes=len(data),sha256=digest(data))
    archive.parent.mkdir(parents=True,exist_ok=True)
    with tarfile.open(archive,'w:gz') as out:
        for name,path in sorted(sources.items()):out.add(path,arcname=name,recursive=False)
        data=json.dumps(manifest,indent=2).encode();info=tarfile.TarInfo('manifest.json');info.size=len(data)
        out.addfile(info,io.BytesIO(data))
    archive.with_suffix(archive.suffix+'.sha256').write_text(digest(archive.read_bytes())+'  '+archive.name+'\n')
    print(json.dumps(dict(archive=str(archive),files=len(sources),bytes=archive.stat().st_size)))


def inspect(archive,workspace,cache,install=False,sha256=None):
    if sha256 and digest(archive.read_bytes())!=sha256:raise ValueError('Archive SHA256 mismatch')
    with tarfile.open(archive,'r:gz') as source:
        members=source.getmembers();names=[m.name for m in members]
        if len(set(names))!=len(names) or any(not m.isfile() for m in members):raise ValueError('Duplicate or non-file archive member')
        manifest=json.load(source.extractfile('manifest.json'))
        if manifest.get('schema')!=1 or manifest.get('version')!='v0.2.0':raise ValueError('Unknown runtime bundle')
        if set(names)!=set(manifest['files'])|{'manifest.json'}:raise ValueError('Manifest member mismatch')
        missing=set('workspace/'+n for n in FILES)-set(names)
        if missing:raise ValueError('Missing runtime files: '+str(missing))
        pending=[]
        for name,entry in manifest['files'].items():
            dest=destination(name,workspace,cache);data=source.extractfile(name).read()
            if len(data)!=entry['bytes'] or digest(data)!=entry['sha256']:raise ValueError('Corrupt member: '+name)
            if dest.exists() and (not dest.is_file() or digest(dest.read_bytes())!=entry['sha256']):
                raise FileExistsError('Will not overwrite different existing asset: '+str(dest))
            pending.append((dest,data))
        if install:
            for dest,data in pending:
                if not dest.exists():
                    dest.parent.mkdir(parents=True,exist_ok=True)
                    with dest.open('xb') as out:out.write(data)
    print(json.dumps(dict(verified=True,installed=install,files=len(pending),runtime_ready=False)))


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['pack','verify','install']);p.add_argument('archive',type=Path)
    p.add_argument('--workspace',type=Path,default=Path('/ws'));p.add_argument('--cache',type=Path,default=Path('/tmp/qda_mujoco_meshes'))
    p.add_argument('--sha256',help='Trusted hash from the release checksum file')
    a=p.parse_args()
    if a.mode=='pack':pack(a.archive,a.workspace.resolve(),a.cache.resolve())
    else:inspect(a.archive,a.workspace.resolve(),a.cache.resolve(),a.mode=='install',a.sha256)


if __name__=='__main__':main()
