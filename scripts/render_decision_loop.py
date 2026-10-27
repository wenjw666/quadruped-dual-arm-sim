"""Render recorded brain I/O beside actual MuJoCo camera frames.

All values come from the take or its matching audit IDs. No robot replay,
fabricated model reasoning, narration, or future results are displayed.
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
    p = argparse.ArgumentParser()
    p.add_argument('--take', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--audit')
    p.add_argument('--fps', type=int, default=15)
    p.add_argument('--preview-wall', type=float)
    p.add_argument('--preview-stage', choices=['input','review','bridge','gate','dispatch','execution','feedback','recheck','complete'])
    a = p.parse_args()
    root, out = Path(a.take), Path(a.out)
    audit = Path(a.audit) if a.audit else root.parent / 'audit'
    out.parent.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((root / 'manifest.json').read_text(encoding='utf8'))
    if manifest['write_errors'] or manifest['dropped_records']:
        raise RuntimeError('Recording has dropped records or write errors')
    events = [json.loads(s) for s in (root / 'events.jsonl').read_text(encoding='utf8').splitlines()]
    ids = {e['data'].get('decision_id') for e in events if e['kind'] == 'decision'}
    present = {e['data'].get('decision_id') for e in events if e['kind'] == 'decision' and e['data']['phase'] == 'model_input'}
    # The first request predates the recorder. Recover its actual, saved input,
    # retaining its original timestamp and explicitly identifying this source.
    for s in (audit / 'events.jsonl').read_text(encoding='utf8').splitlines():
        row = json.loads(s)
        if row.get('decision_id') in ids - present and row['phase'] == 'model_input':
            events.append(dict(kind='decision', wall=row['unix']-manifest['unix_start'], sim=row['sim'], data=row))
            present.add(row['decision_id'])
    by = {}
    for e in sorted(events, key=lambda e: e['wall']):
        by.setdefault(e['kind'], []).append(e)
        if e['kind'] == 'decision':
            by.setdefault('decision:' + e['data']['phase'], []).append(e)
    clocks = {k: [e['wall'] for e in rows] for k, rows in by.items()}
    def latest(kind, t):
        i = bisect.bisect_right(clocks.get(kind, []), t)-1
        return by[kind][i]['data'] if i >= 0 else None
    if not by.get('follow') or not by.get('decision:skill_result'):
        raise RuntimeError('Actual follow camera and terminal execution result are required')
    follow = by['follow']
    fw = np.array([e['wall'] for e in follow])
    fs = np.array([e['sim'] for e in follow])
    if np.any(np.diff(fs) < 0):
        raise RuntimeError('Clock reset within recording')
    start = follow[0]['wall']
    terminal = by['decision:skill_result'][-1]['wall']
    dispatched = by['decision:skill_dispatch'][0]['wall']
    end = min(follow[-1]['wall'], by['decision:model_output'][-1]['wall']+4)
    initial_output=next(e for e in by['decision:model_output'] if e['data'].get('execution_allowed'))
    final_output=by['decision:model_output'][-1]
    final_input=next(e for e in by['decision:model_input'] if e['data']['decision_id']==final_output['data']['decision_id'])
    accepted=next(e for e in by.get('trace',[]) if e['data']['phase']=='submitted')
    initial_input=next(e for e in by['decision:model_input'] if e['data']['decision_id']==initial_output['data']['decision_id'])
    # Semantic-bridge timing is not separately traced. Unpack its saved result
    # only AFTER the aggregate output arrived, with an explicit editorial hold.
    # Do not manufacture a model-call start/return timestamp or reasoning text.
    bridge=initial_output['data']['model'].get('decision_source')=='interpretation_bridge'
    motion_start=accepted['wall']
    motion_seconds=float(np.interp(terminal,fw,fs)-np.interp(motion_start,fw,fs))
    segments=[dict(stage='input',duration=initial_output['wall']-start,begin=start,end=initial_output['wall'],clock='wall')]
    segments.append(dict(stage='review',duration=4,begin=initial_output['wall'],end=initial_output['wall'],clock='hold'))
    if bridge:segments.append(dict(stage='bridge',duration=4,begin=initial_output['wall'],end=initial_output['wall'],clock='hold'))
    segments.extend([
        dict(stage='gate',duration=3,begin=initial_output['wall'],end=initial_output['wall'],clock='hold'),
        dict(stage='dispatch',duration=3,begin=motion_start,end=motion_start,clock='hold'),
        dict(stage='execution',duration=motion_seconds,begin=motion_start,end=terminal,clock='sim'),
        dict(stage='feedback',duration=4,begin=terminal,end=terminal,clock='hold'),
        dict(stage='recheck',duration=final_output['wall']-final_input['wall'],begin=final_input['wall'],end=final_output['wall'],clock='wall'),
        dict(stage='complete',duration=7,begin=final_output['wall'],end=final_output['wall'],clock='hold')])
    total=sum(s['duration'] for s in segments)
    def playback_time(elapsed):
        for s in segments:
            if elapsed<s['duration']:
                if s['clock']=='sim':t=float(np.interp(np.interp(s['begin'],fw,fs)+elapsed,fs,fw))
                elif s['clock']=='wall':t=s['begin']+elapsed
                else:t=s['begin']
                return t,s
            elapsed-=s['duration']
        return segments[-1]['begin'],segments[-1]
    fonts = ['C:/Windows/Fonts/msyh.ttc', '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc']
    font_path = next((s for s in fonts if Path(s).exists()), None)
    if font_path is None:
        raise RuntimeError('A Chinese font is required')
    @lru_cache(24)
    def font(size): return ImageFont.truetype(font_path, size)
    @lru_cache(64)
    def photo(path):
        with Image.open(path) as im:
            return im.convert('RGB')
    colors=dict(bg='#f4f5f2',ink='#182d32',muted='#738184',green='#127963',
                pale='#e2f0e9',amber='#a7630d',warning='#fff0d8',line='#d4ddda')
    def text(d,x,y,value,size=26,color=None):
        d.text((x,y),str(value),font=font(size),fill=color or colors['ink'])
    def arrow(d,points,color,width=3):
        d.line(points,fill=color,width=width,joint='curve')
        x,y=points[-1];px,py=points[-2];angle=math.atan2(y-py,x-px)
        tip=[(x,y),(x-12*math.cos(angle-.45),y-12*math.sin(angle-.45)),
             (x-12*math.cos(angle+.45),y-12*math.sin(angle+.45))]
        d.polygon(tip,fill=color)
    def tick(d,x,y,color=None):
        d.line([(x,y+7),(x+7,y+14),(x+21,y-3)],fill=color or colors['green'],width=4)
    def image_at(canvas,path,box):
        x,y,w,h=box
        im=photo(str(path))
        scale=min(w/im.width,h/im.height)
        im=im.resize((round(im.width*scale),round(im.height*scale)),Image.Resampling.LANCZOS)
        canvas.paste(im,(x+(w-im.width)//2,y+(h-im.height)//2))
    def pill(d,x,y,label,warning=False):
        color=colors['amber'] if warning else colors['green']
        width=math.ceil(d.textlength(label,font=font(21)))+32
        d.rounded_rectangle((x,y,x+width,y+37),radius=18,fill=colors['warning'] if warning else colors['pale'])
        text(d,x+16,y+3,label,21,color)
    def check_row(d,x,y,label,ok=True):
        if ok:tick(d,x+2,y+12)
        else:d.ellipse((x,y+5,x+13,y+18),fill=colors['amber'])
        text(d,x+36,y,label,26,colors['green'] if ok else colors['amber'])
    def code(d,x,y,value):
        d.rounded_rectangle((x,y,x+536,y+94),radius=12,fill=colors['bg'])
        text(d,x+20,y+26,value,28)
    headlines={
        'input':'任务与现场状态触发调用',
        'review':'审查返回与当前状态冲突',
        'bridge':'再次调用模型，转换为技能候选',
        'gate':'程序复查，通过后才允许执行',
        'dispatch':'将确认后的候选下发给机器人',
        'execution':'机器人执行，模块持续回传状态',
        'feedback':'导航结果返回，触发新一轮审查',
        'recheck':'带着新图像和成功结果，再次调用',
        'complete':'模型确认完成，程序核对结果'}
    labels={'input':'本轮输入','review':'调用 1 · 反馈审查','bridge':'调用 2 · 语义转换',
            'gate':'程序校验 · 此处不调用模型','dispatch':'技能指令下发',
            'execution':'执行阶段 · VLM 待命','feedback':'新事件 · 导航到达',
            'recheck':'调用 3 · 完成审查','complete':'闭环完成'}
    previews={s['stage']:s for s in segments}
    def render(t,segment,elapsed=0):
        stage=segment['stage'];held=segment['clock']=='hold'
        canvas=Image.new('RGB',(1920,1080),colors['bg']);d=ImageDraw.Draw(canvas)
        inp=latest('decision:model_input',t);output=latest('decision:model_output',t)
        dispatch=latest('decision:skill_dispatch',t);result=latest('decision:skill_result',t)
        c=inp['context'] if inp else {}
        text(d,40,22,'大脑如何形成执行闭环',37)
        text(d,1420,34,'MuJoCo  ·  B2 + 双 Aubo',23,colors['muted'])
        text(d,40,83,headlines[stage],27)
        # One dominant actual camera; no repeated overview or diagnostic panes.
        imrow=latest('follow',t)
        if imrow:image_at(canvas,root/imrow['file'],(40,130,1200,675))
        d.rounded_rectangle((60,149,202,186),radius=6,fill='#18342f')
        text(d,75,153,'实际仿真',21,'#ffffff')
        clock='结果展开 · 暂停' if held else '物理仿真 1×' if segment['clock']=='sim' else '模型处理实录'
        width=math.ceil(d.textlength(clock,font=font(21)))+32
        d.rounded_rectangle((1220-width,149,1220,186),radius=6,fill='#18342f')
        text(d,1236-width,153,clock,21,'#ffffff')
        d.rounded_rectangle((1280,130,1880,805),radius=18,fill='#ffffff')
        x=1312;maxw=536
        pill(d,x,155,labels[stage],stage=='review')
        titles={'input':'前往南侧入口','review':'原始输出：complete',
                'bridge':'导航 → 南侧入口','gate':'允许下发','dispatch':'执行端已接收',
                'execution':'导航控制正在运行','feedback':'导航成功',
                'recheck':'现在可以结束了吗？','complete':'本轮任务已完成'}
        text(d,x,216,titles[stage],36,colors['amber'] if stage=='review' else colors['ink'])
        if stage in ('input','recheck'):
            image=inp.get('image') if inp else None
            if image:
                image_at(canvas,audit/Path(image['path']).name,(x,293,280,210))
            if stage=='input':
                states=[('底盘就绪',bool(c.get('robot_ready'))),('无活动任务',not c.get('active_task')),
                        ('无故障',not c.get('fault'))]
            else:
                states=[('新图像',bool(c.get('feedback',{}).get('camera_fresh'))),
                        ('导航已成功',c.get('previous_status')=='SUCCEEDED'),
                        ('机器人就绪',bool(c.get('robot_ready')))]
            for i,(name,ok) in enumerate(states):
                d.ellipse((x+305,309+i*63,x+317,321+i*63),fill=colors['green'] if ok else colors['amber'])
                text(d,x+331,298+i*63,name,23)
            text(d,x,532,'图像 + 状态' if stage=='input' else '新图像 + 执行结果',29)
            arrow(d,[(x+268,594),(x+268,632)],colors['green'])
            # Spinner signals pending processing; it is not invented progress.
            d.ellipse((x+118,677,x+130,689),fill=colors['green'])
            text(d,x+147,659,'SmolVLM 处理中',28)
            text(d,x,749,'输入图像为本次调用的实际冻结帧',20,colors['muted'])
        elif stage=='review':
            m=output['model']
            code(d,x,316,m.get('raw_output',''))
            text(d,x,449,'但任务尚未执行',30,colors['amber'])
            text(d,x,502,'这条输出不能直接变成指令',25)
            arrow(d,[(x+268,575),(x+268,614)],colors['amber'])
            text(d,x+99,645,'转入语义转换调用',28)
            text(d,x,749,'按记录展开：审查返回与转换结果',20,colors['muted'])
        elif stage=='bridge':
            m=output['model'];proposal=m['bridge_proposal']
            text(d,x,296,'用户目标 + 同一现场图像',26)
            arrow(d,[(x+268,353),(x+268,392)],colors['green'])
            code(d,x,428,f'{proposal["skill"]}  →  {proposal["target_id"]}')
            matches=proposal==output['candidate']
            check_row(d,x,579,'与用户指定目标一致',matches)
            text(d,x,651,'随后交给程序复查状态',26)
            text(d,x,749,'转换调用的独立耗时未单独记录',20,colors['muted'])
        elif stage=='gate':
            check_row(d,x,323,'模型候选与指定目标一致',output['model'].get('bridge_matches_candidate',True))
            check_row(d,x,411,'前后状态均允许执行',output['guard_expected']==output['guard_current']=='dispatch')
            check_row(d,x,499,'检查通过，生成下发凭据',output['gate_passed'])
            text(d,x,635,'VLM 输出经过检查才放行',28)
            text(d,x,749,'模型返回与机器人执行之间有明确校验',20,colors['muted'])
        elif stage=='dispatch':
            pr=dispatch['proposal']
            code(d,x,318,f'{pr["skill"]}  →  {pr["target_id"]}')
            text(d,x,463,'/robot_skill',32)
            accepted_now=latest('trace',t)
            accepted_ok=bool(accepted_now and accepted_now.get('response',{}).get('accepted'))
            check_row(d,x,546,'执行请求已接受',accepted_ok)
            text(d,x,651,'一次下发，关联同一个任务结果',25)
            text(d,x,749,'执行凭据与任务 ID 已保存在审计记录',20,colors['muted'])
        elif stage=='execution':
            rl=latest('rl',t) or {};owner=latest('owner',t) or {}
            for i,(name,key) in enumerate([('目标速度','desired_velocity'),('实际速度','actual_body_velocity')]):
                v=rl.get(key,[])
                speed=math.hypot(*v[:2]) if len(v)>=2 else None
                yy=307+i*151
                text(d,x,yy,name,24,colors['muted'])
                text(d,x+263,yy-9,f'{speed:.2f} m/s' if speed is not None else '无数据',37)
                d.rounded_rectangle((x,yy+71,x+536,yy+82),radius=5,fill=colors['pale'])
                if speed:
                    d.rounded_rectangle((x,yy+71,x+max(10,min(536,536*speed)),yy+82),radius=5,fill=colors['green'])
            text(d,x,643,'Nav2  →  RL 运控',28)
            text(d,x,698,f'控制权：{owner.get("owner","无数据")}  ·  速度与状态回传',23)
            text(d,x,749,'此段不重复调用 VLM',20,colors['muted'])
        elif stage=='feedback':
            report=result['report'];arrival=report.get('arrival',{})
            code(d,x,315,'SUCCEEDED' if report['success'] else 'FAILED')
            text(d,x,448,'实际到达误差',24,colors['muted'])
            text(d,x,489,f'{arrival["xy_error_m"]*100:.2f} cm',54,colors['green'])
            arrow(d,[(x+268,594),(x+268,632)],colors['green'])
            text(d,x+68,660,'成功结果触发再次审查',29)
            text(d,x,749,'到达 ≠ 直接宣布整个闭环完成',20,colors['muted'])
        elif stage=='complete':
            code(d,x,316,output['model']['raw_output'])
            check_row(d,x,476,'程序核对：任务确已成功',output['guard_current']=='complete' and output['gate_passed'])
            check_row(d,x,553,'模型确认：complete',output['decision']=='complete')
            text(d,x,663,'本轮无需再下发指令',28)
            text(d,x,749,'本例为用户指定导航任务的闭环审查',20,colors['muted'])
        # A single persistent call graph. The return path lights up when the
        # actual terminal event arrives; no continuous VLM call is implied.
        node_y=850;node_w=312;node_h=86
        xs=[80,558,1036,1514]
        active={'input':1,'review':1,'bridge':1,'gate':2,'dispatch':3,
                'execution':3,'feedback':0,'recheck':1,'complete':2}[stage]
        names=['汇总输入','调用 VLM','程序校验','机器人执行']
        sub=['图像 + 当前状态','审查 / 语义转换','目标与状态','导航技能']
        if stage in ('feedback','recheck','complete'):sub[0]='新图像 + 执行结果'
        if stage=='review':sub[1]='调用 1 · 反馈审查'
        if stage=='bridge':sub[1]='调用 2 · 语义转换'
        if stage in ('recheck','complete'):sub[1]='调用 3 · 完成审查'
        for i,xx in enumerate(xs):
            selected=i==active
            d.rounded_rectangle((xx,node_y,xx+node_w,node_y+node_h),radius=14,
                fill=colors['green'] if selected else '#ffffff',
                outline=colors['green'] if selected else colors['line'],width=2)
            tw=d.textlength(names[i],font=font(28))
            text(d,xx+(node_w-tw)/2,node_y+8,names[i],28,'#ffffff' if selected else colors['ink'])
            sw=d.textlength(sub[i],font=font(19))
            text(d,xx+(node_w-sw)/2,node_y+51,sub[i],19,'#d8eee7' if selected else colors['muted'])
            if i<3:
                col=colors['green'] if i+1==active else colors['line']
                arrow(d,[(xx+node_w+17,node_y+43),(xs[i+1]-17,node_y+43)],col,4)
        returning=stage in ('feedback','recheck','complete')
        col=colors['green'] if returning else colors['line']
        arrow(d,[(1670,947),(1670,1009),(236,1009),(236,947)],col,4)
        label='执行结果返回  ·  触发新一轮审查'
        tw=d.textlength(label,font=font(22))
        d.rectangle((960-tw/2-20,987,960+tw/2+20,1032),fill=colors['bg'])
        text(d,960-tw/2,992,label,22,col if returning else colors['muted'])
        text(d,40,1050,'真实图像与接口记录  ·  暂停段用于展开已收到的结果  ·  无配音',17,colors['muted'])
        return canvas
    if a.preview_stage or a.preview_wall is not None:
        if a.preview_stage:
            s=previews[a.preview_stage]
            t=(s['begin']+s['end'])/2
        else:
            t=a.preview_wall
            s=next((s for s in segments if s['begin']<=t<=s['end']),segments[-1])
        render(t,s).save(out);return
    cmd=['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1920x1080','-r',str(a.fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    count=math.ceil(total*a.fps)
    try:
        for i in range(count):
            t,segment=playback_time(i/a.fps)
            proc.stdin.write(render(t,segment,i/a.fps).tobytes())
            if i%(a.fps*10)==0:print(f'{i}/{count}',flush=True)
        proc.stdin.close()
        if proc.wait():raise RuntimeError('FFmpeg failed')
    except Exception:
        proc.kill();proc.wait();raise
    render(final_output['wall'],segments[-1]).save(out.with_suffix('.jpg'))
    out.with_suffix('.json').write_text(json.dumps(dict(take=str(root.resolve()),audit=str(audit.resolve()),source='recorded ROS messages and matching saved model input',clock='wall 1x for recorded model processing; simulation 1x during motion; labelled editorial holds for result inspection',segments=segments,duration=count/a.fps,fps=a.fps,dropped_records=0,readme_approved=False),ensure_ascii=False,indent=2),encoding='utf8')


if __name__=='__main__':main()
