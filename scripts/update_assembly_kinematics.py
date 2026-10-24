from pathlib import Path
p=Path('/ws/src/qda_kinematics/qda_kinematics/kinematics.py');s=p.read_text(encoding='utf-8-sig')
s=s.replace('GRASP_PAD = 0.11','from .assembly import GRIPPER_PAD, GRIPPER_ADAPTER, LEFT_ARM_BASE, RIGHT_ARM_BASE, LEFT_ARM_RPY, RIGHT_ARM_RPY\nGRASP_PAD = GRIPPER_PAD')
s=s.replace('_LEFT_MOUNT = _rpy_matrix(0.0, 0.5, 1.2)','_LEFT_MOUNT = _rpy_matrix(*LEFT_ARM_RPY)').replace('_RIGHT_MOUNT = _rpy_matrix(0.0, 0.5, -1.2)','_RIGHT_MOUNT = _rpy_matrix(*RIGHT_ARM_RPY)')
s=s.replace('np.array([0.16, 0.17, 0.325])','np.array(LEFT_ARM_BASE)').replace('np.array([0.16, -0.17, 0.325])','np.array(RIGHT_ARM_BASE)')
s=s.replace('origin = pos + 0.002 *','origin = pos + GRIPPER_ADAPTER *').replace('deck_c = np.array([0.02, 0.0, 0.265])','deck_c = np.array([0.02, 0.0, 0.125])').replace('np.array([0.30, 0.28, 0.045])','np.array([0.28, 0.21, 0.025])').replace('np.array([0.16, 0.0])','np.array([0.10, 0.0])').replace('(samples[:, :, 2] > 0.30) & (samples[:, :, 2] < 0.56)','(samples[:, :, 2] > 0.15) & (samples[:, :, 2] < 0.54)')
p.write_text(s)
p=Path('/ws/src/qda_kinematics/qda_kinematics/scene.py');s=p.read_text();s=s.replace('LEFT_ARM_BASE = (0.16, 0.17, 0.325)\nRIGHT_ARM_BASE = (0.16, -0.17, 0.325)','from .assembly import LEFT_ARM_BASE, RIGHT_ARM_BASE');p.write_text(s)
p=Path('/ws/src/qda_bringup/qda_bringup/visualization_node.py');s=p.read_text();s=s.replace('from qda_kinematics.kinematics import _aubo_fk, _LEFT_MOUNT','from qda_kinematics.kinematics import _aubo_fk, _LEFT_MOUNT\nfrom qda_kinematics.assembly import LEFT_ARM_BASE');s=s.replace('np.array([0.16, 0.17, 0.325])','np.array(LEFT_ARM_BASE)');p.write_text(s)
