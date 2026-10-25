"""Synthetic Chinese WAV -> offline ASR -> bounded task interpretation."""
import json
from pathlib import Path
from qda_brain.voice import transcribe
from qda_brain.contracts import parse_local


def main():
    root=Path('/ws/output/v2_brain/voice');rows=[]
    expected={'status':'inspect','home':'home','pause':'pause','north':'navigate'}
    for name,skill in expected.items():
        result=transcribe(root/(name+'.wav'));result.update(case=name,expected_skill=skill)
        try:
            result['proposal']=parse_local(result['text'])
            result['passed']=result['proposal']['skill']==skill and result['mean_word_confidence']>=.75
        except ValueError as exc:result.update(passed=False,rejection=str(exc))
        rows.append(result)
    report={'source':'synthetic Windows SAPI files, not microphone or user speech','cases':rows,
            'passed':all(row['passed'] for row in rows)}
    (root.parent/'voice_acceptance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
