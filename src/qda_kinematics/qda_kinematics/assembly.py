"""Single source for the photograph-inspired upper-body assembly (metres/radians)."""
import math
DECK_XYZ = (0.02, 0., 0.105)
DECK_SIZE = (0.54, 0.40, 0.036)
MAST_XYZ = (0.16, 0., 0.151)
SHOULDER_Z = 0.43
LEFT_ARM_BASE = (0.16, 0.105, SHOULDER_Z)
RIGHT_ARM_BASE = (0.16, -0.105, SHOULDER_Z)
LEFT_ARM_RPY = (-math.pi/2, 0., 0.)
RIGHT_ARM_RPY = (math.pi/2, 0., 0.)
CAMERA_XYZ = (0.185, 0., 0.592)
CAMERA_RPY = (0., 0.35, 0.)
LIDAR_XYZ = (-0.14, 0., 0.36)
LIDAR_RPY = (0., 0.4, 0.)
GRIPPER_ADAPTER = 0.008
GRIPPER_PAD = 0.145
GRIPPER_OPEN = 0.0
GRIPPER_GRASP = 0.42
