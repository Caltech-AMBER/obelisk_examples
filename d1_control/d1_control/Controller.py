import numpy as np
from math import pi

from obelisk_control_msgs.msg import PositionSetpoint, VelocityCommand
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

from d1_control.utils.KinematicChain import KinematicChain
from d1_control.constants import *

class Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str="d1_controller") -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.logger = self.get_logger()
        self.declare_ros_parameters()

        self.chain = KinematicChain(node=self, 
                                    baseframe=BASE_FRAME, 
                                    tipframe=TIP_FRAME, 
                                    expectedjointnames=JOINT_NAMES_6_DOF)
        self.q0 = None # initialize the starting joint positions (neglect the gripper)

        self.register_obk_subscription(
            SUB_VCMD_TOPIC,
            self.vcmd_callback, # type: ignore
            msg_type=VelocityCommand,
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.start_time = self.get_clock().now()
        self.vcmd = np.zeros(3)
        self.logger.info("Configured controller")
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg (ObeliskEstimatorMsg): The Obelisk message containing the 
            state estimate of the eight joints representing the arm.
        """
        self._q = np.array(x_hat_msg.q_joints)[:-2]
        if self.q0 is None:
            self.initialize_ik_parameters(self._q)

    def declare_ros_parameters(self) -> None:
        """
        Declare ros parameters such that the parameter values can be passed in 
        from the .yaml file and thereby the command line when launching the 
        node
        """
        # Velocity Limits
        self.declare_parameter("v_x_max", V_X_MAX)
        self.declare_parameter("v_x_min", V_X_MIN)
        self.declare_parameter("v_y_max", V_Y_MAX)
        self.declare_parameter("w_z_max", W_Z_MAX)

    def initialize_ik_parameters(self, q0: np.ndarray) -> None:
        """
        Initialize parameters for computing the inverse kinematics of
        the arm.
        TODO: Initialize position of gripper. Confirm that the tip position
        is in between the gripper.
        """
        self.q0 = q0
        # Initialize the desired position, velocity, and orientation
        (self.p0, self.R0, _, _) = self.chain.fkin(self.q0)
        self.v0 = np.zeros(3)
        
        self.qd = self.q0 # the desired joint positions from the previous time step
        self.pd = self.p0 # the desired task position from the previous time step
        self.Rd = self.R0 # the desired task orientation from the previous time step

        self.J = None # Jacobian used to calculate joint positions

        self._q = self.q0 # current joint positions

        self.logger.info("Initialized inverse kinematics parameters.")

    def get_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def vcmd_callback(self, cmd_msg: VelocityCommand) -> None:
        """
        Update the commanded velocity of the gripper in the x and y direction. # TODO: add z direction.
        Update the commanded angular velocity about the z axis.

        Args:
            cmd_msg
        """
        v_x_min = self.get_param("v_x_min")
        v_x_max = self.get_param("v_x_max")
        self.vcmd[0] = min(max(cmd_msg.v_x, v_x_min), v_x_max)

        v_y_max = self.get_param("v_y_max")
        self.vcmd[1] = min(max(cmd_msg.v_y, -v_y_max), v_y_max)
        
        w_z_max = self.get_param("w_z_max")
        self.vcmd[2] = min(max(cmd_msg.w_z, -w_z_max), w_z_max)

        # self.logger.info("vcmd: %s" % self.vcmd)

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
        if self.q0 is None:
            return
        
        # # Computing the control input
        # # Grab the current time.
        # # `t` doesn't start at 0. It starts at 0.13 seconds.
        # t = self.t - self.start_time.nanoseconds * 1e-9 
        # w = 1

        # if t < pi / w: 
        #     u = np.zeros(8).astype(float).tolist() 
        # else:
        #     u = [(0.3 * np.sin(w * t)) for _ in range(6)]
        #     u_gripper = 0.015 * np.sin(w * t) + 0.015
        #     u.append(u_gripper)
        #     u.append(-u_gripper)

        # # Create the message
        # position_setpoint_msg = PositionSetpoint()
        # position_setpoint_msg.u_mujoco = u
        # position_setpoint_msg.q_des = u
        # self.obk_publishers[PUB_CONTROL_TOPIC].publish(position_setpoint_msg)
        # assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        # return position_setpoint_msg # ignore type checking for now
