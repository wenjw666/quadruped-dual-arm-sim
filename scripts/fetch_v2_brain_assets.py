"""Fetch pinned, public upstream code/models; no account or audio upload."""
import argparse
import concurrent.futures
import hashlib
import json
import time
from pathlib import Path
import requests

HF_REV='72e986006ef53e37cdd3f6d4241c90b0f01df376'
XIAOZHI_REV='ade6cd1cc25ee105dc7d0fee600c56b8dd8b4305'
ASSETS={
 'llama-b11878-bin-ubuntu-x64.tar.gz':('https://github.com/ggml-org/llama.cpp/releases/download/b11878/llama-b11878-bin-ubuntu-x64.tar.gz','edadc40f684c3e44c7d1261eb72c993b98e66c3104555cbc43a0b261d773437f'),
 'SmolVLM-500M-Instruct-Q8_0.gguf':(f'https://huggingface.co/ggml-org/SmolVLM-500M-Instruct-GGUF/resolve/{HF_REV}/SmolVLM-500M-Instruct-Q8_0.gguf?download=true','9d4612de6a42214499e301494a3ecc2be0abdd9de44e663bda63f1152fad1bf4'),
 'mmproj-SmolVLM-500M-Instruct-Q8_0.gguf':(f'https://huggingface.co/ggml-org/SmolVLM-500M-Instruct-GGUF/resolve/{HF_REV}/mmproj-SmolVLM-500M-Instruct-Q8_0.gguf?download=true','d1eb8b6b23979205fdf63703ed10f788131a3f812c7b1f72e0119d5d81295150'),
 'xiaozhi-in-rdk.tar.gz':(f'https://codeload.github.com/D-Robotics/xiaozhi-in-rdk/tar.gz/{XIAOZHI_REV}',None),
 'vosk-model-small-cn-0.22.zip':('https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip',None),
}

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(2**20),b''):h.update(chunk)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',default='/ws/output/v2_brain/assets');p.add_argument('--models-only',action='store_true');p.add_argument('--asset');a=p.parse_args()
    root=Path(a.root);root.mkdir(parents=True,exist_ok=True)
    prior=json.loads((root/'manifest.json').read_text()) if (root/'manifest.json').exists() else {}
    def fetch(item):
        name,(url,expected)=item;path=root/name
        expected=expected or prior.get(name,{}).get('sha256')
        if not path.exists():
            part=path.with_suffix(path.suffix+'.part')
            # Public model hosts support validated byte ranges. Separate bounded
            # pieces avoid restarting a large model after an intermittent TLS EOF.
            if name.endswith(('.gguf','.zip')) or name.startswith('llama-b'):
                ranged_download(url,part)
                part.replace(path)
            for attempt in range(12) if not path.exists() else ():
                try:
                    offset=part.stat().st_size if part.exists() else 0
                    with requests.get(url,headers={'Range':f'bytes={offset}-'} if offset else {},stream=True,timeout=(30,60)) as r:
                        r.raise_for_status()
                        with part.open('ab' if r.status_code==206 else 'wb') as f:
                            for chunk in r.iter_content(65536):f.write(chunk)
                    part.replace(path);break
                except requests.RequestException as exc:
                    print(f'Retry {attempt+1}: {name}: {type(exc).__name__}',flush=True)
                    if attempt==11:raise
                    time.sleep(2)
        actual=digest(path)
        if expected and actual!=expected:raise ValueError('Asset checksum mismatch: '+name)
        print('Verified '+name+' '+str(path.stat().st_size)+' bytes',flush=True)
        return name,dict(url=url,sha256=actual,bytes=path.stat().st_size,upstream_sha256=bool(ASSETS[name][1]))
    selected={k:v for k,v in ASSETS.items() if not a.models_only or k.endswith(('.gguf','.zip'))}
    if a.asset:selected={a.asset:ASSETS[a.asset]}
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        rows=dict(pool.map(fetch,selected.items()))
    rows={**prior,**rows}
    (root/'manifest.json').write_text(json.dumps(rows,indent=2)+'\n')


def ranged_download(url,part):
    for attempt in range(8):
        try:
            with requests.get(url+('&' if '?' in url else '?')+'qda_range=size',headers={'Range':'bytes=0-0'},stream=True,timeout=(30,45)) as response:
                response.raise_for_status()
                if response.status_code!=206:raise RuntimeError('Host does not support byte ranges')
                total=int(response.headers['Content-Range'].split('/')[-1])
            break
        except requests.RequestException:
            if attempt==7:raise
            time.sleep(1)
    pieces=part.parent/(part.name+'.chunks');pieces.mkdir(exist_ok=True);chunk=512*1024
    def piece(start):
        end=min(start+chunk,total)-1;dest=pieces/str(start)
        if dest.exists() and dest.stat().st_size==end-start+1:return dest
        for attempt in range(8):
            try:
                query=url+('&' if '?' in url else '?')+'qda_range='+str(start)
                r=requests.get(query,headers={'Range':f'bytes={start}-{end}'},timeout=(30,60))
                r.raise_for_status()
                if r.status_code!=206 or r.headers.get('Content-Range')!=f'bytes {start}-{end}/{total}' or len(r.content)!=end-start+1:
                    raise RuntimeError('Invalid byte range response')
                dest.write_bytes(r.content);return dest
            except requests.RequestException:
                if attempt==7:raise
                time.sleep(1)
    starts=range(0,total,chunk)
    with concurrent.futures.ThreadPoolExecutor(max_workers=24 if total>200*1024*1024 else 12) as pool:
        files=list(pool.map(piece,starts))
    with part.open('wb') as f:
        for path in files:f.write(path.read_bytes())

if __name__=='__main__':main()
