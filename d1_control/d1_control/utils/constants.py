import numpy as np
from math import pi
import time

"""ROS2 Parameters"""
# Subscribers
SUB_JOY_NAME = "sub_joy_setting"
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
NUM_SERVOS = 7
NUM_CONTROL_INPUTS = 8

QG_INIT = np.array([0, -pi / 3, pi / 3, 0, pi / 6, 0]) # a non-singular joint configuration to initialize to
GRIPPERG_INIT = 0.02 # goal gripper position (meters)

"""Time"""
INIT_TIME = 5 # seconds
MOVING_TIME = 5 # seconds

"""Joystick Parameters"""
MAX_JOY_SPEED = 1
JOY_SPEED_INCREMENT = 0.1

### USE THE FOLLOWING FOR DT = 0.1 ###
V_MAX = 0.3 # m/s
W_MAX = 1 # rad/s

### USE THE FOLLOWING FOR DT = 0.01 ###
# V_MAX = 0.3 # m/s
# W_MAX = 0.3 # rad/s

### USE THE FOLLOWING FOR DT = 0.001 ###
# V_MAX = 1.5 # m/s
# W_MAX = 2 # rad/s

W_MIN = 1e-6 # rad/s

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

# Maximum acceptable displacement between the actual and desired joint positions
JOINT_DISPLACEMENT_THRESHOLD = float('inf')

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
MIN_ERROR_THRESHOLD = 1e-6 # tolerance threshold for stopping iterations
DAMPING_FACTOR = 0.01
# LAMBDA = 1 # Shouldn't exceed 1
MAX_ITERATIONS = 100

"""Recording data"""
RECORDING_STR = "recording"
TIME_STR = time.strftime("%Y%m%d-%H%M%S")
FOLDER_PATH = f"/home/amber-lab/obelisk_examples/d1_control/d1_control/data/{TIME_STR}"

SERVO_COMMAND_FILE_PATH = f"{FOLDER_PATH}/servo_command.csv"
SERVO_STATE_FILE_PATH = f"{FOLDER_PATH}/servo_state.csv"
SERVO_HEADER = ['time'] + [f"servo{i + 1}" for i in range(NUM_SERVOS)]

POSITION_COMMAND_FILE_PATH = f"{FOLDER_PATH}/position_command.csv"
POSITION_STATE_FILE_PATH = f"{FOLDER_PATH}/position_state.csv"
POSITION_HEADER = ['time', 'x', 'y', 'z']