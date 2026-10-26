"""Collect actual sequence/protocol outcomes and preserve failed development trials."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    root=Path('/ws/output/v2_brain')
    def read(name):return json.loads((root/name).read_text())
    def log_pass(name,needle):return needle in (root/name).read_text()
    handoff=read('coordination_handoff_acceptance.json');visual=read('coordination_dual_visual_acceptance.json')
    protocol=read('mission_protocol_acceptance.json');mcp=read('mcp_acceptance.json');servo_gate=read('servo_gate_acceptance.json');shared=read('shared_scene_acceptance.json')
    results=[r['result'] for r in visual['mission']['result']['step_results'] if r['result'].get('skill')=='visual_align']
    runtime=read('runtime_acceptance.json');control=read('control_acceptance.json')
    summary=dict(date_local='2026-10-26',scope='Desktop Docker MuJoCo, fiducial IBVS, navigation goal at spawn',
        handoff=dict(passed=handoff['passed'],mission_id=handoff['mission_id'],completed_steps=handoff['mission']['result']['completed_steps']),
        dual_visual=dict(passed=visual['passed'],mission_id=visual['mission_id'],completed_steps=visual['mission']['result']['completed_steps'],arms=results,
            max_status_receive_gap_wall_s=visual.get('max_status_receive_gap_wall_s')),
        preflight=visual['preflight_rejects_unavailable_step'],external_skill_rejected=visual['external_skill_rejected'],
        replay=visual.get('terminal_replay_unchanged',False),isolated_mission_protocol=protocol['passed'],
        mux=log_pass('coordination_mux.log','PASS:'),paused_clock=log_pass('paused_clock.log','PASS:'),
        delayed_action_cancellation=log_pass('skill_cancellation.log','PASS:'),
        unit_contracts=log_pass('coordination_brain_unit.log','\nOK'),
        unit_tracking_and_leases=log_pass('coordination_unit.log','\nOK'),mcp=mcp['passed'],servo_gate=servo_gate,shared_scene=shared,
        robot_physics_signature=visual.get('final_state',{}).get('robot_physics_signature'),
        final_fault=visual.get('final_state',{}).get('fault'),
        final_runtime=dict(passed=runtime['passed'],cmd_vel_publishers=runtime['cmd_vel_publishers'],model=runtime['model'],
            rviz_running='rviz2' in runtime['node_names'],all_monitored_topics_fresh=all(v['fresh'] for v in runtime['health']['topics'].values())),
        live_cancel_and_replay=control['passed'],
        limits=['Navigation handoff at spawn, not a long-route benchmark','KLT dropout checked with synthetic shifted images, not occlusion statistics',
            'No physical pick-place acceptance','No real microphone or X5 deployment','Model autonomy still gated','No hot process recovery'],
        failed_development_trials=[])
    for name in ('coordination_handoff_frame_mismatch.json','coordination_dual_visual_status_gap.json','coordination_dual_visual_slow_render.json','coordination_dual_visual_ownership.json','coordination_dual_visual_render_lock.json','coordination_dual_visual_lease_queue.json'):
        r=read(name);summary['failed_development_trials'].append(dict(file=name,passed=r['passed'],mission=r.get('mission')))
    assert all(summary[k] for k in ('preflight','external_skill_rejected','replay','isolated_mission_protocol','mux','paused_clock',
                                  'delayed_action_cancellation','unit_contracts','unit_tracking_and_leases','mcp'))
    assert servo_gate['passed'] and servo_gate['lease_survives_without_motor_commands'] and servo_gate['motor_command_expires_independently']
    assert shared['passed']
    assert runtime['passed'] and control['passed']
    assert handoff['passed'] and visual['passed'] and len(results)==2
    target=Path('/ws/docs/validation/v2_coordination_20261026.json');target.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(8,4.2),layout='constrained')
    for side,color in (('left','#1665a4'),('right','#bc571d')):
        rows=[r for r in visual['servo_samples'] if r['arm']==side]
        ax.plot(range(1,len(rows)+1),[r['error_px'] for r in rows],label=side.capitalize()+' wrist',lw=2,color=color)
    ax.axhline(1.5,ls='--',color='#555555',lw=1,label='1.5 px threshold')
    ax.set(xlabel='Processed frame index (each wrist counted separately)',ylabel='Corner RMS error (px)',
           title='One mission: Nav2 handoff → left IBVS → right IBVS → inspect',ylim=(0,None))
    ax.grid(alpha=.18);ax.legend(frameon=False);ax.spines[['top','right']].set_visible(False)
    fig.savefig(root/'coordination_ibvs.png',dpi=160);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ('handoff','dual_visual','mux','paused_clock','delayed_action_cancellation')},indent=2))


if __name__=='__main__':main()
