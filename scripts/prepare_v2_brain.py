"""Verify downloaded assets and unpack local runtimes without cloud activation."""
import hashlib
import json
import tarfile
import zipfile
import os
from pathlib import Path
from fetch_v2_brain_assets import ASSETS,digest


def main():
    root=Path('/ws/output/v2_brain/assets');runtime=Path('/opt/qda-brain-runtime');runtime.mkdir(parents=True,exist_ok=True)
    records={}
    for name in ('llama-b11878-bin-ubuntu-x64.tar.gz','xiaozhi-in-rdk.tar.gz','vosk-model-small-cn-0.22.zip',
                 'SmolVLM-500M-Instruct-Q8_0.gguf','mmproj-SmolVLM-500M-Instruct-Q8_0.gguf'):
        path=root/name
        if not path.exists():raise FileNotFoundError('Download incomplete: '+name)
        actual=digest(path);expected=ASSETS[name][1]
        if expected and expected!=actual:raise ValueError('Checksum mismatch: '+name)
        records[name]={'url':ASSETS[name][0],'sha256':actual,'upstream_sha256':bool(expected),'bytes':path.stat().st_size}
        if name.endswith('.tar.gz'):
            with tarfile.open(path) as archive:
                members=archive.getmembers();top=runtime/members[0].name.split('/')[0]
                if top.exists():
                    # Idempotent while llama-server is running: verify existing
                    # contents instead of truncating a mapped executable/library.
                    for member in members:
                        dest=runtime/member.name
                        if not dest.resolve().is_relative_to(runtime.resolve()):raise ValueError('Archive traversal')
                        if member.isfile():
                            if not dest.is_file() or hashlib.sha256(archive.extractfile(member).read()).hexdigest()!=digest(dest):
                                raise ValueError('Existing runtime differs; stop it and repair: '+str(dest))
                        elif member.issym() and (not dest.is_symlink() or os.readlink(dest)!=member.linkname):
                            raise ValueError('Runtime symlink mismatch: '+str(dest))
                else:archive.extractall(runtime,filter='data')
        elif name.endswith('.zip'):
            with zipfile.ZipFile(path) as archive:
                for info in archive.infolist():
                    if not (root/info.filename).resolve().is_relative_to(root.resolve()):raise ValueError('Archive traversal')
                archive.extractall(root)
    (root/'manifest.json').write_text(json.dumps(records,indent=2)+'\n')
    print('Verified local Q8 model, matching projector, CPU runtime, offline ASR and unactivated official Xiaozhi source.')


if __name__=='__main__':main()
