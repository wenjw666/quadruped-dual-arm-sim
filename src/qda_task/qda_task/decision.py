"""Nonblocking task guards and command ownership, independent of ROS transport.

Drivers must expose start(stage, goal), poll(handle), cancel(handle), and hold().
No controller or gripper command is inferred from a language-model response.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TaskGoal:
    task_id: str
    object_id: str
    target_id: str
    arm: str
    scene_version: int


@dataclass
class Observation:
    stamp: float
    scene_version: int
    base_speed: float = math.inf
    base_tilt: float = math.inf
    object_id: str = ''
    target_id: str = ''
    pose_valid: bool = False
    confidence: float = 0.
    depth_valid: bool = False
    held: bool | None = None
    released: bool | None = None
    gripper_empty: bool | None = None
    arm_retracted: bool = False
    placement_error: float = math.inf
    visual_confirmation: bool = False
    source: str = 'unknown'


class CommandArbiter:
    """One expiring owner per resource; stale lease tokens can never reacquire it."""
    def __init__(self):
        self.leases = {}
        self.epoch = 0

    def acquire(self, resource, owner, now, ttl=.5):
        if resource not in ('base','left_arm','right_arm') or not owner or not math.isfinite(now) or not 0 < ttl <= 1.:
            raise ValueError('Invalid lease')
        current=self.leases.get(resource)
        if current and current[2]>now:raise RuntimeError('RESOURCE_BUSY')
        self.epoch+=1;token=(owner,self.epoch)
        self.leases[resource]=(token,now,now+ttl)
        return token

    def renew(self,resource,token,now,ttl=.5):
        if not 0<ttl<=1. or not self.valid(resource,token,now):raise RuntimeError('STALE_LEASE')
        self.leases[resource]=(token,now,now+ttl)

    def valid(self,resource,token,now):
        lease=self.leases.get(resource)
        return bool(lease and lease[0]==token and lease[1]<=now<lease[2])

    def release(self,resource,token):
        lease=self.leases.get(resource)
        if lease and lease[0]==token:del self.leases[resource]


class DecisionExecutor:
    STAGES=('navigate','localize','pick','transfer','place','verify')
    MOBILE_STAGES=('navigate','transfer')
    TERMINAL=('SUCCESS','FAILED','CANCELED')
    def __init__(self,driver,objects,targets,pose_max_age=.25,stable_duration=1.,max_retries=2,history_limit=256):
        self.driver,self.objects,self.targets=driver,set(objects),set(targets)
        self.pose_max_age,self.stable_duration,self.max_retries=pose_max_age,stable_duration,max_retries
        self.history_limit=history_limit;self.history={};self.active=None;self.handle=None
        self.stable_since=None;self.events=[];self.last_tick=None

    def submit(self,goal,now):
        if not math.isfinite(now) or not goal.task_id or len(goal.task_id)>128 or goal.object_id not in self.objects or goal.target_id not in self.targets or goal.arm not in ('left','right'):
            raise ValueError('INVALID_GOAL')
        previous=self.history.get(goal.task_id)
        if previous:
            if previous['goal']!=goal:raise ValueError('TASK_ID_CONFLICT')
            return dict(previous)
        if self.active:raise RuntimeError('TASK_BUSY')
        if len(self.history)>=self.history_limit:del self.history[next(iter(self.history))]
        self.active=goal.task_id;self.handle=None;self.stable_since=None;self.last_tick=now
        self.history[goal.task_id]=dict(goal=goal,status='RUNNING',index=0,stage='navigate',started=now,
                                        retries=0,error='',skill_succeeded=False)
        self.events.append((now,'ACCEPTED',goal.task_id));return dict(self.history[goal.task_id])

    def _finish(self,status,error,now):
        row=self.history[self.active]
        if self.handle is not None:self.driver.cancel(self.handle)
        self.handle=None;self.driver.hold() # Holding never implies opening a loaded gripper.
        row.update(status=status,error=error);self.events.append((now,status,error));self.active=None
        return dict(row)

    def cancel(self,task_id,now):
        if self.active==task_id:return self._finish('CANCELED','USER_CANCEL',now)
        return dict(self.history[task_id])

    def tick(self,obs,now):
        if not self.active:return None
        row=self.history[self.active];goal=row['goal'];stage=row['stage']
        if not math.isfinite(now) or (self.last_tick is not None and now<self.last_tick):return self._finish('FAILED','CLOCK_ROLLBACK',now)
        self.last_tick=now
        if not math.isfinite(obs.stamp) or not 0<=now-obs.stamp<=self.pose_max_age:
            return self._finish('FAILED','STALE_OBSERVATION',now)
        if obs.scene_version!=goal.scene_version:return self._finish('FAILED','SCENE_CHANGED',now)
        stable=math.isfinite(obs.base_speed) and math.isfinite(obs.base_tilt) and 0<=obs.base_speed<=.02 and abs(obs.base_tilt)<=.10
        self.stable_since=(now if self.stable_since is None else self.stable_since) if stable else None
        stable_long=self.stable_since is not None and now-self.stable_since>=self.stable_duration
        if stage not in self.MOBILE_STAGES and not stable:
            return self._finish('FAILED','BASE_UNSTABLE',now)
        if now-row['started']>(120. if stage in self.MOBILE_STAGES else 30.):return self._finish('FAILED','STAGE_TIMEOUT',now)
        if stage in ('localize','pick','place','verify'):
            valid=(obs.source=='vision' and obs.object_id==goal.object_id and obs.target_id==goal.target_id
                   and obs.pose_valid and obs.depth_valid and math.isfinite(obs.confidence) and .8<=obs.confidence<=1.)
            if not valid:
                if self.handle is not None:return self._finish('FAILED','POSE_LOST_DURING_SKILL',now)
                self.driver.hold();return dict(row)
        needs_held=stage=='transfer' or (stage=='place' and self.handle is None and not row['skill_succeeded'])
        if needs_held and obs.held is not True:return self._finish('FAILED','PAYLOAD_UNCONFIRMED',now)
        if stage not in self.MOBILE_STAGES and not stable_long:return dict(row)
        if self.handle is None and not row['skill_succeeded']:
            self.handle=self.driver.start(stage,goal);self.events.append((now,'SKILL_STARTED',stage))
        if not row['skill_succeeded']:
            status=self.driver.poll(self.handle)
            if status=='RUNNING':return dict(row)
            if status!='SUCCESS':
                # Only perception/planning-only localization is intrinsically safe to retry.
                # A failed pick/place may already have changed physical state.
                if stage=='localize' and row['retries']<self.max_retries:
                    self.driver.cancel(self.handle);self.handle=None;row['retries']+=1;return dict(row)
                return self._finish('FAILED','SKILL_FAILED:'+stage,now)
            self.handle=None;row['skill_succeeded']=True
        passed={'navigate':stable_long,'localize':True,'pick':obs.held is True,
                'transfer':obs.held is True and stable_long,'place':obs.released is True and obs.gripper_empty is True,
                'verify':(obs.visual_confirmation and obs.released is True and obs.gripper_empty is True
                          and obs.arm_retracted and math.isfinite(obs.placement_error) and 0<=obs.placement_error<=.02)}[stage]
        if not passed:return dict(row) # Do not reissue a completed close/open action.
        if stage=='verify':return self._finish('SUCCESS','',now)
        row['index']+=1;row.update(stage=self.STAGES[row['index']],started=now,retries=0,skill_succeeded=False)
        self.events.append((now,'STAGE',row['stage']));return dict(row)
