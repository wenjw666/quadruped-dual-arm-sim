from pathlib import Path
p=Path('/ws/src/qda_kinematics/qda_kinematics/assembly.py');s=p.read_text(encoding='utf-8-sig').replace('MAST_XYZ = (0.10','MAST_XYZ = (0.16').replace('LEFT_ARM_BASE = (0.10','LEFT_ARM_BASE = (0.16').replace('RIGHT_ARM_BASE = (0.10','RIGHT_ARM_BASE = (0.16').replace('CAMERA_XYZ = (0.125','CAMERA_XYZ = (0.185');p.write_text(s)
p=Path('/ws/src/qda_kinematics/qda_kinematics/kinematics.py');s=p.read_text().replace('np.array([0.10, 0.0])','np.array([LEFT_ARM_BASE[0], 0.0])');p.write_text(s)
p=Path('/ws/scripts/test_refined_robot.py');s=p.read_text().replace('[[.64,.32,.01],[.64,.32,.13],[.60,.32,.02]]','[[.64,.32,.01],[.64,.32,.13],[.69,.32,.01]]');p.write_text(s)
