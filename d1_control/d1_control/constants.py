"""ROS2 Topics"""
SUB_VCMD_TOPIC = "sub_vel_cmd_setting"
PUB_CONTROL_TOPIC = "pub_ctrl"

"""Default Velocity Ranges"""
V_X_MAX = 1
V_X_MIN = -1
V_Y_MAX = 0.5
W_Z_MAX = 0.5

"""Kinematic Chain"""
BASE_FRAME = "base_link"
TIP_FRAME = "Link6"
JOINT_NAMES = ["joint0",
               "joint1",
               "joint2",
               "joint3",
               "joint4",
               "joint5",
               "joint6",
               "joint6_2",]
JOINT_NAMES_6_DOF = JOINT_NAMES[:-2]

"""Inverse Kinematics"""
GAMMA = 0.1
LAMBDA = 1