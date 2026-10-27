"""Assemble actual camera frames and recorded ROS state on one time axis.

No robot re-rendering, generated action, narration, or illustrative diagrams.
The small monitoring column displays only received service/topic data. All
inputs remain in the take directory; playback clock and staleness are visible.
"""
import argparse
import bisect
import json
import math
import subprocess
from functools import lru_cache
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--take',required=True);p.add_argument('--out',required=True)
    p.add_argument('--title',default='机器人功能实录')
    p.add_argument('--clock',choices=['sim','wall','hybrid'],default='sim',help='hybrid preserves planning wall time and uses simulation time during motion')
    p.add_argument('--speed',type=float,default=1.)
    p.add_argument('--fps',type=int,default=15);p.add_argument('--preview',action='store_true')
    a=p.parse_args();root=Path(a.take);out=Path(a.out);out.parent.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((root/'manifest.json').read_text()) if (root/'manifest.json').exists() else None
    if manifest is None:
        if not a.preview:raise RuntimeError('Recording must finish before final assembly')
        manifest=dict(write_errors=[],dropped_records=None)
    if manifest['write_errors']:raise RuntimeError(manifest['write_errors'])
    raw=(root/'events.jsonl').read_text(encoding='utf-8')
    rows=raw.splitlines()
    if not raw.endswith('\n'):rows=rows[:-1]
    events=sorted([json.loads(s) for s in rows],key=lambda x:x['wall'])
    if manifest.get('unix_start'):
        for e in events:
            if e['kind']=='trace' and e['data'].get('unix'):e['wall']=e['data']['unix']-manifest['unix_start']
        events.sort(key=lambda e:e['wall'])
    by={}
    for e in events:by.setdefault(e['kind'],[]).append(e)
    if not by.get('follow'):raise RuntimeError('Missing actual follow camera; do not substitute a pose replay')
    walls={k:[e['wall'] for e in rows] for k,rows in by.items()}
    def latest(kind,t):
        n=bisect.bisect_right(walls.get(kind,[]),t)-1
        return by[kind][n] if n>=0 else None
    trace=by.get('trace',[])
    start=max(by['follow'][0]['wall'],(trace[0]['wall']-2) if trace else 0)
    finishes=[e['wall'] for e in trace if e['data']['phase'] in ('finished','failed')]
    end=min(by['follow'][-1]['wall'],finishes[-1]+4 if finishes else by['follow'][-1]['wall'])
    samples=by['follow'];fw=np.array([e['wall'] for e in samples]);fs=np.array([e['sim'] for e in samples])
    if np.any(np.diff(fs)<0):raise RuntimeError('Simulation clock reset within take')
    submitted=[e['wall'] for e in trace if e['data']['phase']=='submitted']
    split=min(end,max(start,submitted[0] if submitted else end))
    planning_duration=split-start
    split_sim=float(np.interp(split,fw,fs))
    t0,t1=(float(np.interp(start,fw,fs)),float(np.interp(end,fw,fs))) if a.clock=='sim' else (start,end)
    duration=(planning_duration+(float(np.interp(end,fw,fs))-split_sim)/a.speed) if a.clock=='hybrid' else (t1-t0)/a.speed
    count=max(1,math.ceil(duration*a.fps))
    font_path='C:/Windows/Fonts/msyh.ttc'
    if not Path(font_path).exists():font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    @lru_cache(16)
    def font(size):return ImageFont.truetype(font_path,size)
    @lru_cache(48)
    def photo(name):
        with Image.open(root/name) as im:return im.convert('RGB')
    def text(draw,xy,s,size=24,color='#e3e9ed'):draw.text(xy,str(s),font=font(size),fill=color)
    def lines(draw,x,y,s,width=24,size=24,color='#e3e9ed'):
        # Chinese text and the bounded short labels are wrapped, never elided.
        s=str(s)
        for i in range(0,len(s),width):text(draw,(x,y),s[i:i+width],size,color);y+=size+9
        return y
    def tile(canvas,draw,kind,t,box,label):
        row=latest(kind,t);x,y,w,h=box
        if row:
            im=photo(row['data']['file']);ratio=min(w/im.width,h/im.height)
            im=im.resize((round(im.width*ratio),round(im.height*ratio)),Image.Resampling.BILINEAR)
            canvas.paste(im,(x+(w-im.width)//2,y+(h-im.height)//2))
            age=t-row['wall'];label+=f' | {row["sim"]:.2f}s' if row['sim'] is not None else ' | RECEIVE TIME'
            if age>5:label+=f' | AGE {age:.1f}s'
        else:label+=' | NO FRAME'
        draw.rectangle((x,y,x+w,y+30),fill='#142029');text(draw,(x+8,y+2),label,20)
    names={'navigate':'NAVIGATE','home':'ARM HOME','visual_align':'VISUAL ALIGN','inspect':'INSPECT','pick_place':'PICK CARRY PLACE'}
    phases={'input':'INPUT','interpreting':'MODEL REQUEST','proposal':'MODEL PROPOSAL',
            'transcribing':'ASR','recognized':'ASR RESULT','validated_plan':'PLAN VALIDATED',
            'submitted':'DISPATCHED','execution':'EXECUTING','finished':'FINISHED','failed':'STOP / FAILED'}
    def frame(t):
        canvas=Image.new('RGB',(1920,1080),'#10171e');d=ImageDraw.Draw(canvas)
        sim=float(np.interp(t,fw,fs))
        text(d,(14,3),a.title,23)
        sim_clock=a.clock=='sim' or (a.clock=='hybrid' and t>=split)
        speed=1. if a.clock=='hybrid' and t<split else a.speed
        text(d,(740,3),f'{"SIM CLOCK" if sim_clock else "WALL CLOCK"} {speed:g}x | sim {sim:.2f}s | record {t:.1f}s',21)
        tile(canvas,d,'follow',t,(0,36,1440,810),'MuJoCo PHYSICS / LIVE TAKE')
        fourth=('servo','VISUAL SERVO') if latest('servo',t) else ('perception','RGB-D DETECTION') if latest('perception',t) else ('front','HEAD CAMERA')
        for i,(key,label) in enumerate([('overview','OVERVIEW'),('left','LEFT WRIST'),('right','RIGHT WRIST'),fourth]):
            tile(canvas,d,key,t,(i*360,846,360,234),label)
        x=1460;y=16;text(d,(x,y),'BRAIN / EXECUTION MONITOR',27);y+=48
        tr=latest('trace',t)
        if tr:
            data=tr['data'];text(d,(x,y),phases.get(data['phase'],data['phase']),23,'#69dbc4');y+=38
            asr=[e for e in trace if e['wall']<=t and e['data']['phase']=='recognized']
            if asr:
                text(d,(x,y),'OFFLINE WAV: '+asr[-1]['data']['response'].get('text',''),22);y+=34
            inp=[e for e in trace if e['wall']<=t and e['data']['phase']=='input']
            if inp:
                sentences=inp[-1]['data'].get('sentences') or ([inp[-1]['data']['text']] if inp[-1]['data'].get('text') else [])
                sentences=[{'前往北侧入口':'NAVIGATE TO NORTH ENTRY','前往南侧入口':'NAVIGATE TO SOUTH ENTRY'}.get(s,s) for s in sentences]
                for sentence in sentences:y=lines(d,x,y,'· '+sentence,18,22)
            y+=12
            proposals=[e for e in trace if e['wall']<=t and e['data']['phase']=='proposal']
            if proposals:
                res=proposals[-1]['data']['response'];pr=res.get('proposal') or res.get('candidate') or {}
                text(d,(x,y),'LATEST MODEL PROPOSAL',22);y+=32
                y=lines(d,x,y,res.get('source','REJECTED'),25,19,'#a8b8c6')
                for k in ('skill','arm','target_id'):
                    text(d,(x,y),k+': '+str(pr.get(k,'—')),22);y+=30
                text(d,(x,y),'SEMANTIC GATE: '+('PASS' if res.get('execution_allowed') else 'REJECT'),22);y+=35
            plans=[e for e in trace if e['wall']<=t and e['data']['phase']=='validated_plan']
            if plans:
                text(d,(x,y),'VALIDATED SEQUENCE',23);y+=35
                run=[e for e in trace if e['wall']<=t and e['data']['phase']=='execution']
                mission=run[-1]['data']['mission'] if run else {};result=mission.get('result',{})
                done=result.get('completed_steps',0);current=result.get('current_step',0)
                for i,step in enumerate(plans[-1]['data']['plan']['steps']):
                    state='DONE' if i<done else 'RUNNING' if i==current and mission.get('status')=='RUNNING' else 'WAIT'
                    arm={'left':'L ','right':'R ','none':''}[step['arm']]
                    text(d,(x,y),f'{i+1}  {arm}{names.get(step["skill"],step["skill"])}  · {state}',22,'#69dbc4' if i<done else '#e3e9ed');y+=32
                if mission:text(d,(x,y),mission['status'],23,'#69dbc4' if mission['status']=='SUCCEEDED' else '#e3e9ed');y+=35
            if data['phase']=='failed':y=lines(d,x,y,data.get('error',''),30,17,'#ffbc80')
        else:text(d,(x,y),'WAITING FOR INPUT',23)
        metric=latest('metrics',t)
        if metric:
            mdata=metric['data'];side={'left':'LEFT WRIST','right':'RIGHT WRIST'}.get(mdata.get('arm'),'WRIST')
            text(d,(x,680),f'{side} ERROR {mdata.get("error_px",0):.2f} px',24,'#69dbc4')
            text(d,(x,714),mdata.get('source',''),18,'#a8b8c6')
        else:
            pose=latest('pick_pose',t)
            if pose:
                pdata=pose['data'];xyz=pdata['xyz']
                text(d,(x,680),'RGB + DEPTH / '+pdata['frame'],21,'#69dbc4')
                text(d,(x,714),f'x {xyz[0]:.2f}  y {xyz[1]:.2f}  z {xyz[2]:.2f} m',21)
        # Every module panel is built from the received topic samples. This is
        # deliberately a compact diagnostic table rather than a decorative
        # animation: stale/missing values remain visible as such.
        y=748;text(d,(x,y),'FEEDBACK BUS / MODULE STATUS',22,'#f2c66d');y+=30
        health=latest('health',t);brain=latest('skill',t);rl=latest('rl',t);manip=latest('manipulation',t);owner=latest('owner',t);labels=latest('detections',t)
        def module(name,value,ok=True):
            nonlocal y
            state='LIVE' if ok and value is not None else 'STALE / MISSING'
            color='#69dbc4' if state=='LIVE' else '#ffbc80'
            text(d,(x,y),f'{name:<10} {state}',18,color);y+=24
        module('RGB-D',labels,labels is not None);module('RL locomotion',rl,rl is not None);module('brain',brain,brain is not None);module('mission',latest('mission',t),latest('mission',t) is not None);module('manipulation',manip,manip is not None);module('owner',owner,owner is not None)
        if health:
            h=health['data'];fresh=sum(1 for row in h.get('topics',{}).values() if row.get('fresh'));total=len(h.get('topics',{}))
            text(d,(x,y),f'HEALTH {fresh}/{total} topics fresh | camera {"fresh" if h.get("front_camera_fresh") else "stale"}',18,'#a8b8c6');y+=27
        if rl:
            r=rl['data'];text(d,(x,y),f'RL v=[{", ".join(f"{v:.2f}" for v in r.get("actual_body_velocity",[]))}] fault={r.get("fault") or "none"}',17,'#a8b8c6');y+=27
        if manip:
            m=manip['data'];text(d,(x,y),f'physical={m.get("stage","-")} carrying={m.get("carrying",False)} error={m.get("error") or "none"}',17,'#a8b8c6');y+=27
        # A real OccupancyGrid, Nav2 path and measured base position.
        m=latest('map',t)
        if m:
            meta=m['data'];im=photo(meta['file']).copy();md=ImageDraw.Draw(im)
            def pixel(point):return ((point[0]-meta['origin'][0])/meta['resolution'],(point[1]-meta['origin'][1])/meta['resolution'])
            path=latest('plan',t)
            if path and len(path['data'])>1:md.line([pixel(p) for p in path['data']],fill='#16a4ee',width=2)
            j=latest('joints',t)
            if j and j['data']['base'].get('position'):
                px,py=pixel(j['data']['base']['position']);md.ellipse((px-3,py-3,px+3,py+3),fill='#e37427')
            known=np.any(np.array(photo(meta['file']))!=115,axis=2)
            yy,xx=np.where(known)
            if len(xx):im=im.crop((max(0,int(xx.min())-10),max(0,int(yy.min())-10),min(im.width,int(xx.max())+11),min(im.height,int(yy.max())+11)))
            im=im.transpose(Image.Transpose.FLIP_TOP_BOTTOM);im.thumbnail((440,230))
            canvas.paste(im,(x+(440-im.width)//2,790))
            text(d,(x,755),'LIVE GRID / PLAN / BASE',20)
        text(d,(1460,1036),'Operator candidate; runtime feedback',18,'#a8b8c6')
        return canvas
    if a.preview:
        frame((start+end)/2).save(out);return
    cmd=['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1920x1080','-r',str(a.fps),'-i','-',
         '-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    try:
        for n in range(count):
            clock=t0+n/a.fps*a.speed
            if a.clock=='hybrid':
                elapsed=n/a.fps
                wall=start+elapsed if elapsed<planning_duration else float(np.interp(split_sim+(elapsed-planning_duration)*a.speed,fs,fw))
            else:wall=float(np.interp(clock,fs,fw)) if a.clock=='sim' else clock
            proc.stdin.write(frame(wall).tobytes())
            if n%(a.fps*10)==0:print(f'{n}/{count} frames',flush=True)
        hold=frame(end);hd=ImageDraw.Draw(hold)
        hd.rectangle((700,0,1440,35),fill='#10171e');text(hd,(740,3),'FINAL HOLD | LAST RECEIVED STATE',21)
        for _ in range(a.fps*2):proc.stdin.write(hold.tobytes())
        proc.stdin.close()
        if proc.wait()!=0:raise RuntimeError('FFmpeg failed')
    except Exception:
        proc.kill();proc.wait();raise
    out.with_suffix('.json').write_text(json.dumps(dict(take=str(root),source='actual received images; no pose replay',
        playback_clock=a.clock,speed=a.speed,planning_wall_seconds=planning_duration if a.clock=='hybrid' else None,
        wall_range=[start,end],sim_range=[float(np.interp(start,fw,fs)),float(np.interp(end,fw,fs))],
        duration=count/a.fps+2,final_hold_seconds=2,fps=a.fps,dropped_records=manifest['dropped_records'],readme_approved=False),indent=2),encoding='utf-8')
    frame((start+end)/2).save(out.with_suffix('.jpg'))


if __name__=='__main__':main()
