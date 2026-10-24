"""Summarize the latest measured run per profile, without pooled success claims."""
import csv,json
from pathlib import Path
root=Path('/ws/output/mujoco/runs');out=Path('/ws/output/expanded_demo');selected={}
for path in sorted(Path('/ws/output').glob('**/runs/*/summary.json'),key=lambda p:p.parent.name):
    row=json.loads(path.read_text());profile=row['scenario']
    if profile in ('south','north','side','auto'):selected[profile]=(path,row)
rows=[]
config_hashes={}
for profile in ('south','north','side','auto'):
    if profile not in selected:continue
    path,r=selected[profile];audit=path.with_name('clearance_audit.json')
    fingerprints=path.with_name('source_sha256.json')
    if fingerprints.exists():
        for source,digest in json.loads(fingerprints.read_text()).items():
            if source.endswith(('nav2_quad.yaml','kinematics.py','pick_place_node.py','detector_node.py','assembly.py')):
                config_hashes.setdefault(source,{})[profile]=digest
    clear=json.loads(audit.read_text()) if audit.exists() else {}
    events=[json.loads(line)['text'] for line in path.with_name('events.jsonl').read_text().splitlines()]
    pick='RGB-D' if any('感知到抓取目标' in x for x in events) else 'configured fallback'
    place='configured fallback'
    for text in events:
        if '感知到放置标记' in text:place='RGB-D'
        if '未看到绿色标记' in text:place='configured fallback'
    accepted=r['status']=='TASK_SUCCESS' and clear.get('footprint_overlap_samples')==0 and not path.with_name('runtime_overrides.json').exists()
    rows.append(dict(scenario=profile,status=r['status'],acceptance='PASS' if accepted else 'FAIL_OR_UNAUDITED',path_m=round(r['path_m'],3),
        sim_s=round(r['sim_s'],2),wall_s=round(r['wall_s'],2),planning_calls=r['planning_calls'],
        planning_failures=r['planning_failures'],planning_wall_s=round(r['planning_wall_s'],3),
        final_error_m=None if r['final_error_m'] is None else round(r['final_error_m'],4),
        min_reference_clearance_m=clear.get('min_reference_clearance_m'),
        footprint_overlap_samples=clear.get('footprint_overlap_samples'),pick_source=pick,place_source=place,
        run_directory=str(path.parent)))
(out/'acceptance_results.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
(out/'configuration_consistency.json').write_text(json.dumps(config_hashes,indent=2))
if rows:
    with (out/'acceptance_results.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
lines=['# 扩展工厂实际验收记录','',
       '每个场景只列最近一次实测；这是功能验收，不是统计成功率。调试期间任务、导航、感知与显示有版本变化，不据此比较墙钟速度。完整配置与事件见各运行目录。','',
       '| 场景 | 任务 / 验收 | 行程 m | 仿真耗时 s | 放置误差 m | 规划失败/调用 | 抓取/末次放置定位 | 轮廓重叠采样 |',
       '|---|---|---:|---:|---:|---:|---|---:|']
for r in rows:
    lines.append(f"| {r['scenario']} | {r['status']} / {r['acceptance']} | {r['path_m']} | {r['sim_s']} | {r['final_error_m']} | {r['planning_failures']}/{r['planning_calls']} | {r['pick_source']} / {r['place_source']} | {r['footprint_overlap_samples']} |")
lines+=['','指标边界：导航使用真值 odom，机器人为规定运动，夹持为虚拟连接。轮廓审计不覆盖整个机械臂、不测接触力；RGB-D 与配置坐标回退分别标明。','',
        '固定位姿渲染的红块 RGB-D 反投影测试误差约 0.0266 m（表面测点与物体中心之间包含几何偏差）。该测试用于验证图像、深度、内外参链路，不等同于实际任务的落点误差。','']
if len(rows)==4 and config_hashes and all(len(d)==4 and len(set(d.values()))==1 for d in config_hashes.values()):
    lines+=['最终四场景的导航参数、运动学、任务节点、感知节点和装配参数源码指纹一致，详见 `configuration_consistency.json`。','']
else:
    lines+=['所列运行的源码版本并非全部相同；`configuration_consistency.json` 列出各模块指纹。不能把分阶段功能验证当作最终同版本四场景基准；统一重复测试请使用 `run_scenarios.ps1`。','']
for r in rows:lines.append(f"- {r['scenario']}: `{r['run_directory']}`")
(out/'ACCEPTANCE.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(json.dumps(rows,ensure_ascii=False,indent=2))
