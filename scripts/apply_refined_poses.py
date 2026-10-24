import json,re
from pathlib import Path
poses=json.load(open('/ws/output/model_refinement_v1/poses.json'))
p=Path('/ws/src/qda_kinematics/qda_kinematics/kinematics.py');s=p.read_text()
for key,name in [('left','AUBO_HOME_LEFT'),('right','AUBO_HOME_RIGHT'),('carry','AUBO_CARRY_LEFT')]:
 s=re.sub(name+r' = \([^\n]+\)',name+' = '+str(tuple(round(x,9) for x in poses[key])),s)
s=s.replace('FINGER_CLOSED = 0.001','FINGER_CLOSED = 0.8').replace('# Opening is 7 mm + 2*q. 0.018 leaves a 43 mm gap around the 40 mm cube.','# Robotiq linkage: q=0 opens 85 mm; q=0.42 opens approximately 43 mm.')
s=s.replace('# Mounts are mirrors across the robot center plane. Odd joints flip under that\n# mirror and even joints do not, so this home uses only the even joints and is\n# identical on both arms.','# Photograph-inspired horizontal shoulders; calibrated relaxed display/home poses.')
s=s.replace('# Existing pad IK, target (0.38, 0.25, 0.48) in base_link. Unlike HOME,\n# this folds the payload over the deck instead of 0.72 m out to the side.','# Calibrated carry pad target (0.29, 0.32, 0.28) in base_link.')
p.write_text(s)
