"""Generate reviewable plans against the installed scene/target registry."""
import argparse
import json
from pathlib import Path
from qda_brain.contracts import SCENE_VERSION
from qda_brain.mission import validate_plan


def step(skill,arm='none',target='none'):
    return dict(skill=skill,object_id='servo_marker' if skill=='visual_align' else 'none',target_id=target,arm=arm)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--preset',choices=['handoff','dual_visual','inspection'],default='handoff')
    parser.add_argument('--out',required=True);args=parser.parse_args()
    plans={'handoff':[step('navigate',target='start'),step('home','left'),step('home','right'),step('inspect')],
           'dual_visual':[step('navigate',target='start'),step('visual_align','left'),step('visual_align','right'),step('inspect')],
           'inspection':[step('navigate',target='inspection'),step('inspect'),step('navigate',target='start')]}
    plan=validate_plan(dict(scene_version=SCENE_VERSION,steps=plans[args.preset]))
    target=Path(args.out);target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(path=str(target),plan=plan),ensure_ascii=False))


if __name__=='__main__':main()
