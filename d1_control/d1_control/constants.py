import numpy as np
from math import pi

"""ROS2 Parameters"""
# Subscribers
SUB_VCMD_NAME = "sub_vcmd_setting"
SUB_GOAL_NAME = "sub_goal_setting"

# Publishers
PUB_CONTROL_NAME = "pub_ctrl"

# Other parameters
TIMER_CTRL_NAME = "timer_ctrl_setting"
TIMER_PERIOD_SEC_KEY = "timer_period_sec"

"""Robot info"""
URDF_FILENAME = "/home/amber-lab/obelisk/obelisk_ws/src/robots/d1_description/urdf/d1.urdf"
NUM_JOINTS = 6
JOINT_ID = 6
Q_INIT = np.array([0, -pi / 3, pi / 3, 0, pi / 6, 0]) # a non-singular joint configuration to initialize to

"""Time"""
INIT_TIME = 5 # seconds

"""Default Velocity Ranges"""
V_X_MAX = 1 # m/s FIXME
V_X_MIN = -1
V_Y_MAX = 0.5
W_Z_MAX = 0.5

"""Control Limits"""
# Unit: radians
JOINT_LIMITS = np.array([
    [-135, 135],
    [-90, 90],
    [-90, 90],
    [-135, 135],
    [-90, 90],
    [-135, 135]
]) * pi / 180

# Unit: meters
GRIPPER_LIMITS = np.array([0, 0.03])

"""Kinematic Chain"""
BASE_FRAME = "base_link"
TIP_FRAME = "Link6"
JOINT_NAMES = ["joint1",
               "joint2",
               "joint3",
               "joint4",
               "joint5",
               "joint6"]
GRIPPER_NAMES = ["gripper1", "gripper2"]

"""Inverse Kinematics"""
MIN_ERROR_THRESHOLD = 1e-4 # tolerance threshold for stopping iterations
MAX_ERROR_THRESHOLD = 1
DAMPING_FACTOR = 0.01
# LAMBDA = 1 # Shouldn't exceed 1
# MAX_ITERATIONS = 1000