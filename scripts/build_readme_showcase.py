"""Build a GitHub-compatible showcase from existing, measured simulation footage.

No invented frames, narration or task results. Requires ffmpeg and Pillow.
"""
from pathlib import Path
import subprocess
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'media/readme'
WORK = ROOT / 'output/readme-showcase'
WORK.mkdir(parents=True, exist_ok=True)


def font(size):
    for path in ('C:/Windows/Fonts/segoeuib.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def build():
    # Viewer: robotics developers; promise: see the actual task; mechanism:
    # contact grasp + RL carry + place; next step: open the full recording.
    strip = Image.new('RGB', (960, 76), '#0b1220')
    d = ImageDraw.Draw(strip)
    d.rectangle((0, 0, 5, 76), fill='#2dd4bf')
    d.text((24, 11), 'QUADRUPED / DUAL ARM', font=font(25), fill='#f1f5f9')
    d.text((25, 46), 'CONTACT GRASP   /   RL CARRY   /   PLACE', font=font(12), fill='#94a3b8')
    d.rounded_rectangle((766, 21, 937, 54), radius=16, fill='#123c39')
    d.ellipse((780, 33, 788, 41), fill='#2dd4bf')
    d.text((799, 27), 'SIMULATION', font=font(15), fill='#5eead4')
    strip.save(WORK / 'strip.png')
    filters = (
        '[0:v]setpts=(PTS-STARTPTS)/5,fps=8,scale=960:-2:flags=lanczos[sim];'
        '[1:v][sim]vstack=inputs=2:shortest=1,fps=5,scale=800:-2:flags=lanczos,split[a][b];'
        '[a]palettegen=max_colors=80:stats_mode=diff[p];'
        '[b][p]paletteuse=dither=bayer:bayer_scale=3'
    )
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-threads', '2',
                    '-i', str(ROOT / 'Videos/physical-pick-carry-place-validated.mp4'),
                    '-loop', '1', '-i', str(WORK / 'strip.png'),
                    '-filter_complex_threads', '1', '-filter_complex', filters,
                    '-t', '24.48', '-loop', '0', str(OUT / 'hero-task.gif')], check=True)
    with Image.open(OUT / 'hero-task.gif') as im:
        assert im.n_frames > 100
        im.convert('RGB').save(WORK / 'hero-preview.png')
        print({'asset': 'hero-task.gif', 'size': im.size, 'frames': im.n_frames,
               'bytes': (OUT / 'hero-task.gif').stat().st_size})


if __name__ == '__main__':
    build()
