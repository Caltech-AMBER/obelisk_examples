from d1_control.constants import *

def limit_joints(joints: list) -> None:
        """Modify the joints list such that they are within the joint limits."""
        for i, input in enumerate(joints):
            min_num = JOINT_LIMITS[i][0]
            max_num = JOINT_LIMITS[i][1]
            joints[i] = limit(input, min_num, max_num)

def limit_grippers(grippers: list) -> None:
    """Modify the gripper list such that they are within the gripper limits."""
    min_num = GRIPPER_LIMITS[0]
    max_num = GRIPPER_LIMITS[1]
    grippers[0] = limit(grippers[0], min_num, max_num)
    grippers[1] = -grippers[0]

def limit(num: float, min_num: float, max_num: float) -> float:
    """Limit `num` to be between `min_num` and `max_num`."""
    return max(min(max_num, num), min_num)