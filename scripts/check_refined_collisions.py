import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity,GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from qda_kinematics.kinematics import AUBO_CARRY_LEFT,AUBO_HOME_LEFT,AUBO_HOME_RIGHT,left_grasp_joints
rclpy.init();n=Node('refined_collision_check')
c=n.create_client(GetStateValidity,'/check_state_validity');g=n.create_client(GetPlanningScene,'/get_planning_scene')
assert c.wait_for_service(timeout_sec=90) and g.wait_for_service(timeout_sec=15)
def call(client,request):
 f=client.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=15);assert f.done();return f.result()
r=GetPlanningScene.Request();r.components.components=PlanningSceneComponents.ROBOT_STATE
state=call(g,r).scene.robot_state
names=state.joint_state.name;home=dict(zip(names,state.joint_state.position));results=[]
for side,pose in [('left',AUBO_HOME_LEFT),('right',AUBO_HOME_RIGHT)]:home.update({f'{side}_joint_link{i+1}':float(q) for i,q in enumerate(pose)})
def check(name,pose=None):
 values=dict(home)
 if pose is not None:values.update({f'left_joint_link{i+1}':float(q) for i,q in enumerate(pose)})
 state.joint_state.position=[float(values[x]) for x in names]
 req=GetStateValidity.Request();req.robot_state=state;req.group_name=''
 res=call(c,req)
 row={'name':name,'valid':res.valid,'contacts':sorted(set((x.contact_body_1,x.contact_body_2) for x in res.contacts))};results.append(row);print(row,flush=True)
 return res.valid
assert check('home');assert check('carry',AUBO_CARRY_LEFT)
valid_grasps=[check('grasp_'+str(i),q) for i,q in enumerate(left_grasp_joints([.64,.32,.02],count=12))]
assert any(valid_grasps),'No collision-free grasp branch'
Path('/ws/output/model_refinement_v1/collision_checks.json').write_text(json.dumps(results,indent=2))
print('HOME_CARRY_AND_GRASP_BRANCH_PASS',flush=True)
n.destroy_node();rclpy.shutdown()

