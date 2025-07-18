from typing import Union
import numpy as np
from d1_control.constants import *

def limit_joints(joints: Union[list[float], np.ndarray]) -> None:
    """Modify the joints list/array such that it is within the joint limits."""
    for i, input in enumerate(joints):
        min_num = JOINT_LIMITS[i][0]
        max_num = JOINT_LIMITS[i][1]
        joints[i] = limit(input, min_num, max_num)

def limit_gripper(gripper: float) -> float:
    """Return the gripper position such that it is within its limits."""
    min_num = GRIPPER_LIMITS[0]
    max_num = GRIPPER_LIMITS[1]
    gripper = limit(gripper, min_num, max_num)
    return gripper

def limit(num: float, min_num: float, max_num: float) -> float:
    """Limit `num` to be between `min_num` and `max_num`."""
    return max(min(max_num, num), min_num)