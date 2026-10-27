"""Build the approved-format functional recording from explicit successful takes.

Run only after motion recording finishes. No titles-only slides or narration.
"""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--brain',required=True);p.add_argument('--navigation',required=True)
    p.add_argument('--servo',required=True);p.add_argument('--out-dir',required=True);a=p.parse_args()
    root=Path(__file__).resolve().parents[1];out=Path(a.out_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    chapters=[('01-navigation',a.navigation,'01 / Nav2 + 四足 RL · 到点检查与返回','sim',1.),
              ('02-servo',a.servo,'02 / 固定起点 · 双臂标定板视觉对准','sim',.5),
              ('03-brain',a.brain,'03 / 大脑提案 → 计划校验 → 双臂回位','hybrid',.5)]
    evidence=[];clips=[]
    for name,take,title,clock,speed in chapters:
        take=Path(take).resolve();report=take.with_name(take.name+'_result.json')
        r=json.loads(report.read_text(encoding='utf-8'));m=json.loads((take/'manifest.json').read_text())
        if not r.get('passed') or r.get('mission',{}).get('status')!='SUCCEEDED':raise RuntimeError('Take did not succeed: '+str(take))
        if m['write_errors'] or m['dropped_records']:raise RuntimeError('Recorder loss: '+str(take))
        silent=out/(name+'-silent.mp4');clip=out/(name+'.mp4')
        subprocess.run([sys.executable,str(root/'scripts/render_live_demonstration.py'),'--take',str(take),'--out',str(silent),
                        '--title',title,'--clock',clock,'--speed',str(speed)],check=True)
        if name=='03-brain':
            # The only audio is the exact offline test input, not a voiceover.
            wav=root/'output/v2_brain/voice/status.wav'
            data=json.loads(silent.with_suffix('.json').read_text())
            voice=next(e for e in r['events'] if e['phase']=='transcribing')
            delay=max(0,round((voice['unix']-m['unix_start']-data['wall_range'][0])*1000))
            cmd=['ffmpeg','-y','-v','error','-i',str(silent),'-i',str(wav),'-filter_complex',f'[1:a]adelay={delay}:all=1,apad[a]',
                 '-map','0:v','-map','[a]','-c:v','copy','-c:a','aac','-ar','48000','-ac','2','-shortest','-movflags','+faststart',str(clip)]
        else:
            cmd=['ffmpeg','-y','-v','error','-i',str(silent),'-f','lavfi','-i','anullsrc=channel_layout=stereo:sample_rate=48000',
                 '-map','0:v','-map','1:a','-c:v','copy','-c:a','aac','-ar','48000','-ac','2','-shortest','-movflags','+faststart',str(clip)]
        subprocess.run(cmd,check=True);clips.append(clip)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-of','json',str(clip)]))
        evidence.append(dict(chapter=title,take=str(take),report_sha256=hashlib.sha256(report.read_bytes()).hexdigest(),
            mission_id=r['mission']['task_id'],steps=r['mission']['result']['completed_steps'],duration=float(probe['format']['duration']),
            recording=m,playback=json.loads(silent.with_suffix('.json').read_text())))
    listing=out/'concat.txt';listing.write_text(''.join("file '"+p.as_posix()+"'\n" for p in clips),encoding='utf-8')
    meta=';FFMETADATA1\n';at=0
    for e in evidence:
        end=at+round(e['duration']*1000)
        meta+=f'[CHAPTER]\nTIMEBASE=1/1000\nSTART={at}\nEND={end}\ntitle={e["chapter"]}\n';at=end
    metadata=out/'chapters.ffmeta';metadata.write_text(meta,encoding='utf-8')
    final=out/'qda-functional-demo-review.mp4'
    subprocess.run(['ffmpeg','-y','-v','error','-f','concat','-safe','0','-i',str(listing),'-i',str(metadata),
                    '-map','0','-map_metadata','1','-map_chapters','1','-c','copy','-movflags','+faststart',str(final)],check=True)
    attempts=[]
    for path in sorted(Path(a.servo).resolve().parent.glob('*_result.json')):
        attempt=json.loads(path.read_text(encoding='utf-8'));mission=attempt.get('mission',{})
        failed=[e for e in attempt.get('events',[]) if e.get('phase')=='failed']
        attempts.append(dict(report=str(path),passed=attempt.get('passed'),mode=attempt.get('mode'),
            status=mission.get('status'),completed_steps=mission.get('result',{}).get('completed_steps'),
            error=mission.get('result',{}).get('error_code') or (failed[-1].get('error') if failed else None)))
    (out/'evidence.json').write_text(json.dumps(dict(chapters=evidence,attempts=attempts,video=str(final),sha256=hashlib.sha256(final.read_bytes()).hexdigest(),
        speech='Only the original synthetic status.wav test input; no narration',
        limitations=['Ordered operator inputs, single-skill model proposals; not free-form autonomous planning',
                     'Experimental RL; truth odometry; fiducial visual servo; no physical grasp claim',
                     'Servo chapter starts from its own fixed initial state; navigation-to-servo robustness is not accepted',
                     'Recorded camera reception gaps are preserved; no generated/interpolated robot motion'],
        readme_publish_approved=False),ensure_ascii=False,indent=2),encoding='utf-8')
    print(final,flush=True)


if __name__=='__main__':main()
