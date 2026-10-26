"""Probe the final delivery and compare exact decoded end-hold frames."""
import hashlib
import json
import subprocess
from pathlib import Path
from PIL import Image,ImageDraw
import numpy as np

ROOT=Path(__file__).resolve().parent

def main():
    plan=json.loads((ROOT/'src/data/choreography-plan.json').read_text(encoding='utf-8'))
    movie=ROOT/'out/qda-v2-product-demo.mp4'
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(movie)]))
    (ROOT/'qc/media-probe.json').write_text(json.dumps(probe,indent=2),encoding='utf-8')
    video=next(s for s in probe['streams'] if s['codec_type']=='video');audio=next(s for s in probe['streams'] if s['codec_type']=='audio')
    assert (video['width'],video['height'],video['r_frame_rate'],video['codec_name'])==(1920,1080,'30/1','h264')
    assert audio['codec_name']=='aac' and abs(float(probe['format']['duration'])-plan['frames']/30)<.2
    assert 'black_start:' not in (ROOT/'qc/blackdetect.log').read_text(encoding='utf-8',errors='replace')
    def decoded(frame):return subprocess.check_output(['ffmpeg','-v','error','-ss',str(frame/30),'-i',str(movie),'-frames:v','1','-f','rawvideo','-pix_fmt','rgb24','pipe:1'])
    first,last=plan['frames']-45,plan['frames']-2
    x,y=decoded(first),decoded(last)
    source_a=Image.open(ROOT/'qc/stills/hold-a.png').tobytes();source_b=Image.open(ROOT/'qc/stills/hold-b.png').tobytes()
    assert source_a==source_b,'Source end hold differs pixel-for-pixel'
    delta=np.abs(np.frombuffer(x,dtype=np.uint8).astype(float)-np.frombuffer(y,dtype=np.uint8))
    assert delta.mean()<.01 and delta.max()<=3,'Encoded end hold visibly differs'
    report=dict(delivery=str(movie.name),sha256=hashlib.sha256(movie.read_bytes()).hexdigest(),
        duration_seconds=float(probe['format']['duration']),width=1920,height=1080,fps=30,video_codec='h264',audio_codec='aac',
        bytes=movie.stat().st_size,semanticActionResult='inspected against actual task data',persistentObjectHandoff='QDA rail and progress line preserved',
        motionWindowReview='sampled decoded frame windows; not a real-time performance test',cueSync='WAV sample count per line',
        finalHold=dict(frameA=first,frameB=last,source_pixels_identical=True,encoded_pixels_identical=x==y,
                       encoded_mean_abs_delta=float(delta.mean()),encoded_max_delta=float(delta.max())),blackFramesDetected=False,
        notes=['12 chapter maximum-information frames visually inspected; map and result footer clipping repaired',
               'Narration is offline synthetic Mandarin; no music or real microphone',
               'Raw clips predecoded at 15 fps, output 30 fps; original media preserved',
               'GUI and diagrams are labelled; current task does not validate physical grasp'],readme_publish_approved=False)
    (ROOT/'qc/workflow-evidence.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    canvas=Image.new('RGB',(1440,1160),'#f2f1eb');draw=ImageDraw.Draw(canvas)
    for i,s in enumerate(plan['scenes']):
        frame=s['from']+int(s['frames']*.55)
        im=Image.frombytes('RGB',(1920,1080),decoded(frame)).resize((480,270))
        x0=i%3*480;y0=i//3*290;canvas.paste(im,(x0,y0));draw.text((x0+12,y0+272),f'{i+1} / {s["id"]}',fill='#16242b')
    canvas.save(ROOT/'qc/contact-sheet.jpg',quality=93)
    # Dense consecutive samples around causal/transition windows, with real timestamps.
    points=[(14+i*.5) for i in range(6)]+[(106+i*.5) for i in range(6)]+[(126+i*.5) for i in range(6)]
    sheet=Image.new('RGB',(1440,1740),'#f2f1eb');draw=ImageDraw.Draw(sheet)
    for i,t in enumerate(points):
        im=Image.frombytes('RGB',(1920,1080),decoded(int(t*30))).resize((480,270))
        x0=i%3*480;y0=i//3*290;sheet.paste(im,(x0,y0));draw.text((x0+12,y0+272),f'{t:.1f} s',fill='#16242b')
    sheet.save(ROOT/'qc/motion-windows.jpg',quality=93)
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
