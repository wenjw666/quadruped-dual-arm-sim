"""Collect measured results and plot the two independent visual-servo trials."""
import json
import statistics
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    root=Path('/ws/output/v2_brain');read=lambda name:json.loads((root/name).read_text())
    left=read('ros_acceptance.json');right=read('ros_acceptance_right.json')
    model=read('model_acceptance.json');voice=read('voice_acceptance.json')
    shared=read('shared_scene_acceptance.json');gate=read('servo_gate_acceptance.json');control=read('control_acceptance.json');mcp=read('mcp_acceptance.json')
    world_probe=read('servo_probe.json')
    summary={'date_local':'2026-10-26','backend':'MuJoCo RL + bounded upper-limb PD',
        'left_ibvs':left['align']['result'],'right_ibvs':right['align']['result'],
        'shared_scene':shared,'shared_scene_followup':{k:v for k,v in world_probe.items() if k not in ('trajectories','metrics','modes')},
        'servo_gate':gate,'task_cancel_replay_and_capability_gate':control['passed'],'mcp':mcp['passed'],
        'voice_task_completed':control['voice_task'][0]['status']=='SUCCEEDED',
        'model':{'model':model['model'],'correct':sum(r['passed'] for r in model['cases']),'total':len(model['cases']),
            'median_text_latency_s':statistics.median(r['latency_seconds'] for r in model['cases'] if 'text' in r),
            'failures':[{'text':r.get('text',r.get('case')),'error':r.get('error','wrong intent')} for r in model['cases'] if not r['passed']],
            'autonomous_execution_ready':False},
        'offline_voice':{'correct':sum(r['passed'] for r in voice['cases']),'total':len(voice['cases']),
            'source':voice['source'],'failures':[{'case':r['case'],'transcript':r['text'],'rejection':r.get('rejection')} for r in voice['cases'] if not r['passed']]},
        'limits':['No Xiaozhi cloud/device activation','No microphone measurement','No RDK X5 performance measurement',
                  'Calibration-board IBVS, no markerless grasp','Physical pick-place gated','Navigation uses simulation truth',
                  'No complete MTC/BT or action hot recovery'],
        'model_assets':read('assets/manifest.json')}
    target=Path('/ws/docs/validation/v2_brain_20261026.json');target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(8,4.3),layout='constrained')
    for name,report,color in [('Left wrist',left,'#1665a4'),('Right wrist',right,'#bc571d')]:
        samples=report['servo_samples'];t0=samples[0]['sim_time']
        ax.plot([s['sim_time']-t0 for s in samples],[s['error_px'] for s in samples],label=name,color=color,lw=2)
    ax.axhline(1.5,color='#555555',ls='--',lw=1,label='Acceptance: <1.5 px for four frames')
    ax.set(xlabel='Simulation time since first observation (s)',ylabel='Corner RMS error (px)',
           title='MuJoCo rendered pixels → IBVS → MoveIt Servo → motor torques',ylim=(0,None))
    ax.grid(alpha=.18);ax.legend(frameon=False);ax.spines[['top','right']].set_visible(False)
    fig.savefig(root/'ibvs_validation.png',dpi=160);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('left_ibvs','right_ibvs','model','offline_voice')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
