"""Build frame-accurate captions and presentation data from recorded evidence."""
import argparse
import hashlib
import json
import math
import shutil
import subprocess
import wave
from pathlib import Path

ROOT=Path(__file__).resolve().parent
WS=ROOT.parents[1]

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,value):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    p=argparse.ArgumentParser();p.add_argument('--take',default='take02');p.add_argument('--timeline-only',action='store_true');a=p.parse_args()
    scenes=read(ROOT/'script.json');cursor=0;subtitles=[]
    for s in scenes:
        s['from']=cursor;s['cues']=[];local=24
        for i,line in enumerate(s['lines']):
            name=f'narration/{s["id"]}-{i}.wav'
            with wave.open(str(ROOT/'public'/name)) as wav:duration=math.ceil(wav.getnframes()/wav.getframerate()*30)
            s['cues'].append(dict(fromFrame=local,frames=duration,file=name,text=line))
            subtitles.append((cursor+local,cursor+local+duration,line));local+=duration+10
        s['frames']=max(240,local+25);cursor+=s['frames']
    write(ROOT/'src/data/choreography-plan.json',dict(fps=30,width=1920,height=1080,frames=cursor,scenes=scenes,
        authority='Each caption is timed to its own local SAPI WAV, measured by sample count; no microphone input.',
        invariant='QDA wordmark, chapter rail, semantic colors and progress line remain mounted.'))
    def tc(frame):
        ms=round(frame/30*1000);return f'{ms//3600000:02}:{ms//60000%60:02}:{ms//1000%60:02},{ms%1000:03}'
    (ROOT/'out').mkdir(exist_ok=True)
    (ROOT/'out/qda-v2-zh.srt').write_text('\n\n'.join(f'{i+1}\n{tc(b)} --> {tc(e)}\n{t}' for i,(b,e,t) in enumerate(subtitles)),encoding='utf-8')
    print('Narrated timeline seconds:',cursor/30)
    if a.timeline_only:return
    evidence=WS/'output/product_demo';take=evidence/a.take
    recording=read(take/'manifest.json')
    if recording.get('write_errors'):raise ValueError('Recording has write errors')
    if recording.get('dropped_records',0):raise ValueError('Recording dropped messages; review this take before making a film')
    report=read(evidence/f'dual_visual_{a.take}.json');assert report['passed'],report['mission']['result']
    events=[json.loads(x) for x in (take/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    public=ROOT/'public';public.mkdir(exist_ok=True)
    assets=[]
    def copy(source,name):
        shutil.copy2(source,public/name);assets.append(dict(file=name,source=str(source.relative_to(WS)).replace('\\','/'),sha256=hashlib.sha256((public/name).read_bytes()).hexdigest()))
    for src,name in [('output/model_refinement_v1/perspective.png','robot.png'),('output/model_refinement_v1/upper_detail.png','upper.png'),
        ('output/model_refinement_v1/gripper_detail.png','gripper.png'),('output/v2_audit/b2_contact_preview.mp4','rl.mp4'),
        ('output/v2_brain/voice/status.wav','voice-status.wav')]:copy(WS/src,name)
    for key in ('front','c0','overview','left','right'):
        matches=[x for x in events if x['kind']==key]
        if matches:copy(take/matches[len(matches)//2]['data']['file'],key+'.jpg')
    arm_data={}
    for side in ('left','right'):
        metrics=[x for x in events if x['kind']=='metrics' and x['data'].get('arm')==side]
        if not metrics:raise ValueError('Missing recorded '+side+' metrics')
        begin,end=metrics[0]['wall'],metrics[-1]['wall']
        frames=[x for x in events if x['kind']==side and begin-1<=x['wall']<=end+1]
        clip=public/(side+'-frames');clip.mkdir(exist_ok=True)
        # One sample per actual image; fixed-rate montage clearly labelled as time-compressed.
        for i,row in enumerate(frames):shutil.copy2(take/row['data']['file'],clip/f'{i:06d}.jpg')
        subprocess.run(['ffmpeg','-y','-v','error','-framerate',str(len(frames)/14),'-i',str(clip/'%06d.jpg'),
            '-r','30','-c:v','libx264','-pix_fmt','yuv420p',str(public/(side+'.mp4'))],check=True)
        arm_data[side]=dict(errors=[x['data']['error_px'] for x in metrics],frames=len(frames),wall_range=[begin,end],
            initial=metrics[0]['data']['error_px'],final=metrics[-1]['data']['error_px'])
    nav=read(WS/'output/v2_scene/integration_final.json');scene=read(WS/'output/v2_scene/scene_manifest.json')
    model=read(WS/'output/v2_brain/model_acceptance.json');voice=read(WS/'output/v2_brain/voice_acceptance.json')
    with wave.open(str(public/'voice-status.wav')) as wav:
        import numpy as np
        samples=np.frombuffer(wav.readframes(wav.getnframes()),dtype=np.int16).astype(float)/32768
        waveform=[float(np.max(np.abs(x))) for x in np.array_split(samples,90)]
    write(ROOT/'src/data/evidence.json',dict(mission_id=report['mission_id'],mission=report['mission']['result'],arms=arm_data,
        nav=dict(goal=nav['navigation']['goal'],error=nav['navigation']['position_error_m'],trajectory=nav['trajectory_xy'][::8]),
        scene=scene,voice=voice,model_cases=len(model['cases']),waveform=waveform,source_take=a.take))
    write(ROOT/'qc/source-manifest.json',dict(assets=assets,take=str(take.relative_to(WS)),mission=report['mission_id'],
        raw_camera_clips='actual RGB frames, uniformly retimed to 14 seconds; not real-time latency',
        presentation_pose_replay='recorded joints + truth odom, camera restaged; not a new physics run',
        voice='Windows SAPI synthetic Mandarin; no cloned voice, no microphone',readme_publish_approved=False))


if __name__=='__main__':main()
