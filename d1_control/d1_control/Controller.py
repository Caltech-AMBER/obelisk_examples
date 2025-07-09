import numpy as np
from math import pi

from obelisk_control_msgs.msg import PositionSetpoint, VelocityCommand
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

SUB_VEL_CMD_TOPIC = "sub_vel_cmd_setting"
PUB_CONTROL_TOPIC = "pub_ctrl"

class Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str="d1_controller") -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.start_time = self.get_clock().now() # TODO: Delete
        # Declare parameters such that the parameter values can be passed in 
        # from the .yaml file and thereby the command line when launching the 
        # node.

        # Velocity Limits
        self.declare_parameter("v_x_max", 1.0)
        self.declare_parameter("v_x_min", -1.0)
        self.declare_parameter("v_y_max", 0.5)
        self.declare_parameter("w_z_max", 0.5)

        self.register_obk_subscription(
            SUB_VEL_CMD_TOPIC,
            self.vel_cmd_callback, # type: ignore
            msg_type=VelocityCommand,
        )

        self.received_x_hat = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)

        self.cmd_vel = np.zeros((3,))

        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg (ObeliskEstimatorMsg): The Obelisk message containing the 
            state estimate.
        """
        self.received_x_hat = True

    def vel_cmd_callback(self, cmd_msg: VelocityCommand):
        v_x_min = self.get_parameter("v_x_min").get_parameter_value().double_value
        v_x_max = self.get_parameter("v_x_max").get_parameter_value().double_value
        self.cmd_vel[0] = min(max(cmd_msg.v_x, v_x_min), v_x_max)

        v_y_max = self.get_parameter("v_y_max").get_parameter_value().double_value
        self.cmd_vel[1] = min(max(cmd_msg.v_y, -v_y_max), v_y_max)
        
        w_z_max = self.get_parameter("w_z_max").get_parameter_value().double_value
        self.cmd_vel[2] = min(max(cmd_msg.w_z, -w_z_max), w_z_max)

    def compute_control(self) -> ObeliskControlMsg:
        """
        Compute the joint positions for the 6-DOF+1 robot. 
        
        joint0 to joint5 positions are in radians.
        joint6 and joint6_2 are in meters. 
        They control the gripper.
        The position of motor 6 is positive. 
        The position of motor 6_2 is the negative of that of motor 6.
        The control message contains eight joints for Mujoco to simulate the 
        robot.
        
        Returns:
            obelisk_control_msg (ObeliskControlMsg): The control message.
        """
        if not self.received_x_hat:
            return
        
        # Computing the control input
        # Grab the current time.
        # `t` doesn't start at 0. It starts at 0.13 seconds.
        t = self.t - self.start_time.nanoseconds * 1e-9 
        w = 1

        if t < pi / w: 
            u = np.zeros(8).astype(float).tolist() 
        else:
            u = [(0.3 * np.sin(w * t)) for _ in range(6)]
            u_gripper = 0.015 * np.sin(w * t) + 0.015
            u.append(u_gripper)
            u.append(-u_gripper)

        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = u
        position_setpoint_msg.q_des = u
        self.obk_publishers[PUB_CONTROL_TOPIC].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # ignore type checking for now
