"""Bounded semantic commands. Model text never becomes joint or shell commands."""
import hashlib
import json
import re
import sqlite3
import threading
import time
import math
from qda_kinematics.scene import scene_manifest,PLACE_MARKER_XYZ,PLACE_YAW,APPROACH_IN_BASE,SPAWN_XYZ,SPAWN_YAW

SKILLS=('navigate','home','inspect','visual_align','pick_place','pause')
TARGETS={'start':(*SPAWN_XYZ[:2],SPAWN_YAW),'inspection':(-.2,1.,0.),'north_entry':(-.25,2.25,0.),'south_entry':(.1,-1.85,0.),
         'pick_station':(.14,-.16,0.),'place_station':(
             PLACE_MARKER_XYZ[0]-math.cos(PLACE_YAW)*APPROACH_IN_BASE[0]+math.sin(PLACE_YAW)*APPROACH_IN_BASE[1],
             PLACE_MARKER_XYZ[1]-math.sin(PLACE_YAW)*APPROACH_IN_BASE[0]-math.cos(PLACE_YAW)*APPROACH_IN_BASE[1],PLACE_YAW)}
OBJECTS=('none','red_cube','servo_marker')
SCENE_VERSION=int(hashlib.sha256(json.dumps({'scene':scene_manifest(),'targets':TARGETS},sort_keys=True).encode()).hexdigest()[:8],16)
SCHEMA={'type':'object','additionalProperties':False,'required':['skill','object_id','target_id','arm'],
 'properties':{'skill':{'type':'string','enum':list(SKILLS)},'object_id':{'type':'string','enum':list(OBJECTS)},
               'target_id':{'type':'string','enum':['none',*TARGETS]},'arm':{'type':'string','enum':['none','left','right']}}}

def validate(proposal):
    if not isinstance(proposal,dict) or set(proposal)!=set(SCHEMA['required']):raise ValueError('INVALID_FIELDS')
    for key,spec in SCHEMA['properties'].items():
        if proposal[key] not in spec['enum']:raise ValueError('UNKNOWN_'+key.upper())
    skill=proposal['skill']
    if skill in ('inspect','pause') and any(proposal[k]!='none' for k in ('object_id','target_id','arm')):raise ValueError('UNEXPECTED_SKILL_ARGUMENTS')
    if skill=='home' and (proposal['object_id']!='none' or proposal['target_id']!='none'):raise ValueError('UNEXPECTED_HOME_ARGUMENTS')
    if skill=='navigate' and (proposal['object_id']!='none' or proposal['arm']!='none'):raise ValueError('UNEXPECTED_NAV_ARGUMENTS')
    if skill=='navigate' and proposal['target_id'] not in TARGETS:raise ValueError('MISSING_NAV_TARGET')
    if skill in ('home','visual_align','pick_place') and proposal['arm'] not in ('left','right'):raise ValueError('MISSING_ARM')
    if skill=='visual_align' and proposal['object_id']!='servo_marker':raise ValueError('ALIGN_REQUIRES_FIDUCIAL')
    if skill=='visual_align' and proposal['target_id']!='none':raise ValueError('UNEXPECTED_ALIGN_TARGET')
    if skill=='pick_place' and (proposal['object_id']!='red_cube' or proposal['target_id']!='place_station'):
        raise ValueError('UNSUPPORTED_PICK_PLACE')
    return dict(proposal)

def parse_local(text):
    """Explicit limited-command alternative, labelled rules rather than VLM."""
    t=text.strip().lower().replace(' ','')
    if (not t or len(t)>500 or re.search(r'\b(?:then|and|not|ignore)\b',text.lower())
        or any(w in t for w in ('不要','别','忽略','然后','之后','同时','并且','以及','再',';','；'))):raise ValueError('AMBIGUOUS_OR_NEGATED_REQUEST')
    families=[any(w in t for w in words) for words in (('回位','回到初始','home'),('对准','align'),
        ('导航','前往','去','navigate','goto'))]
    families.append(any(w in t for w in ('查询状态','查看状态','当前状态')) or bool(re.search(r'\b(?:status|inspect)\b',text.lower())))
    if sum(families)>1:raise ValueError('MULTIPLE_SKILLS_REQUIRE_EXPLICIT_PLAN')
    if any(w in t for w in ('左','left')) and any(w in t for w in ('右','right')):raise ValueError('AMBIGUOUS_ARM')
    p=dict(skill='inspect',object_id='none',target_id='none',arm='none')
    if t in ('暂停','停止','停下','pause','stop'):p['skill']='pause'
    elif t in ('查询状态','查看状态','当前状态','status','inspect'):pass
    elif ('回位' in t or '回到初始' in t or 'home' in t):
        p.update(skill='home',arm='left' if '左' in t or 'left' in t else 'right' if '右' in t or 'right' in t else 'none')
    elif ('对准' in t or 'align' in t):
        p.update(skill='visual_align',object_id='servo_marker',arm='left' if '左' in t or 'left' in t else 'right' if '右' in t or 'right' in t else 'none')
    elif any(w in t for w in ('导航','前往','去','navigate','goto')):
        targets=[key for key,words in {'start':('起点','start'),'inspection':('检查点','inspection'),'north_entry':('北侧','north'),
            'south_entry':('南侧','south'),'pick_station':('抓取台','pickstation'),'place_station':('放置台','placestation')}.items() if any(w in t for w in words)]
        if len(targets)!=1:raise ValueError('AMBIGUOUS_NAV_TARGET')
        p.update(skill='navigate',target_id=targets[0])
    elif '红色' in t and any(w in t for w in ('抓','搬','放')):
        p.update(skill='pick_place',object_id='red_cube',target_id='place_station',
                 arm='left' if '左' in t else 'right' if '右' in t else 'none')
    else:raise ValueError('UNSUPPORTED_LOCAL_PHRASE')
    return validate(p)

class Journal:
    def __init__(self,path):
        from pathlib import Path
        Path(path).parent.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock();self.db=sqlite3.connect(path,check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, goal TEXT, status TEXT, result TEXT, updated REAL)')
        self.db.execute("UPDATE tasks SET status='INTERRUPTED', result=? WHERE status IN ('RUNNING','RESERVED')",(json.dumps({'success':False,'error_code':'PROCESS_RESTART'}),));self.db.commit()
    def reserve(self,task_id,goal):
        if not isinstance(task_id,str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,128}',task_id):raise ValueError('INVALID_TASK_ID')
        encoded=json.dumps(goal,sort_keys=True)
        with self.lock:
            row=self.db.execute('SELECT goal,status,result FROM tasks WHERE id=?',(task_id,)).fetchone()
            if row:
                if row[0]!=encoded:raise ValueError('TASK_ID_CONFLICT')
                return {'duplicate':True,'status':row[1],'result':json.loads(row[2])}
            if self.db.execute("SELECT 1 FROM tasks WHERE status IN ('RUNNING','RESERVED')").fetchone():raise ValueError('TASK_BUSY')
            self.db.execute('INSERT INTO tasks VALUES (?,?,?,?,?)',(task_id,encoded,'RESERVED','{}',time.time()));self.db.commit()
            return {'duplicate':False,'status':'RESERVED'}
    def update(self,task_id,status,result=None):
        with self.lock:
            self.db.execute('UPDATE tasks SET status=?,result=?,updated=? WHERE id=?',(status,json.dumps(result or {}),time.time(),task_id));self.db.commit()
    def get(self,task_id=None):
        with self.lock:
            rows=self.db.execute('SELECT id,goal,status,result FROM tasks '+('WHERE id=?' if task_id else 'ORDER BY updated DESC LIMIT 20'),(task_id,) if task_id else ()).fetchall()
        return [dict(task_id=r[0],goal=json.loads(r[1]),status=r[2],result=json.loads(r[3])) for r in rows]
