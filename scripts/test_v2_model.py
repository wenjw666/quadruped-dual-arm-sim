"""Measure real local model semantics/latency; schema validity alone is insufficient."""
import json
import time
from pathlib import Path
from qda_brain.model_client import interpret


def main():
    cases=[('Show robot status','inspect','none'),('查询状态','inspect','none'),
           ('Navigate to the north entry','navigate','none'),('前往北侧','navigate','none'),
           ('Move the left arm home','home','left'),('左臂回位','home','left'),
           ('Pause the robot','pause','none'),
           ('Return the right arm to its starting position','home','right'),
           ('让右边的机械臂回到初始位置','home','right'),
           ('Please go to the southern entry','navigate','none'),
           ('请去南侧通道','navigate','none')]
    rows=[]
    for text,skill,arm in cases:
        start=time.monotonic()
        try:
            row=interpret(text);p=row['proposal'];row['passed']=p['skill']==skill and p['arm']==arm
            if skill=='navigate':row['passed']=row['passed'] and p['target_id']==('south_entry' if 'south' in text or '南' in text else 'north_entry')
        except Exception as exc:row={'passed':False,'error':str(exc),'latency_seconds':time.monotonic()-start}
        row['text']=text;rows.append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
    image=Path('/ws/output/v2_scene/head_rgb.png')
    if not image.exists():image=Path('/ws/output/v2_brain/left_before.png')
    try:
        row=interpret('Show robot status',image.read_bytes());row['passed']=row['proposal']['skill']=='inspect';row['case']='image_input'
    except Exception as exc:row={'case':'image_input','passed':False,'error':str(exc)}
    rows.append(row)
    report={'model':'SmolVLM-500M-Instruct Q8_0 + matching Q8 projector','backend':'llama.cpp CPU',
            'cases':rows,'passed':all(r['passed'] for r in rows),'runtime_ready':False,
            'note':'Small local functional set, not X5 latency or semantic robustness acceptance.'}
    Path('/ws/output/v2_brain/model_acceptance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
