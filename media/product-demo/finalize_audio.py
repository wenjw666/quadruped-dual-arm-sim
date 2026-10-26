"""Two-pass measured narration normalization, preserving encoded video bytes."""
import json
import re
import subprocess
from pathlib import Path

root=Path(__file__).resolve().parent
source=root/'out/qda-v2-product-demo-raw.mp4'
target=root/'out/qda-v2-product-demo.mp4'
measure=subprocess.run(['ffmpeg','-v','info','-i',str(source),'-af','loudnorm=I=-16:TP=-1.5:LRA=11:print_format=json',
    '-vn','-f','null','-'],capture_output=True,text=True,check=True)
(root/'qc/audio-measurement.log').write_text(measure.stderr,encoding='utf-8')
match=re.search(r'\{\s*"input_i".*?\}',measure.stderr,re.S)
if not match:raise RuntimeError('No measured loudness from FFmpeg')
data=json.loads(match.group())
effect=('loudnorm=I=-16:TP=-1.5:LRA=11:measured_I='+data['input_i']+':measured_TP='+data['input_tp']+
        ':measured_LRA='+data['input_lra']+':measured_thresh='+data['input_thresh']+':offset='+data['target_offset']+
        ':linear=false:print_format=json')
result=subprocess.run(['ffmpeg','-y','-v','info','-i',str(source),'-c:v','copy','-af',effect,'-ar','48000',
    '-c:a','aac','-b:a','192k','-movflags','+faststart',str(target)],capture_output=True,text=True,check=True)
(root/'qc/audio-normalization-final.log').write_text(result.stderr,encoding='utf-8')
print('Final narration normalized from measured input; video stream copied.')
