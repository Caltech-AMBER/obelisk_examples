import numpy as np
from math import pi

from obelisk_control_msgs.msg import PositionSetpoint, VelocityCommand
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

from d1_control.constants import *

from d1_control.utils.KinematicChain import KinematicChain
from d1_control.utils.ControlUtils import limit_joints, limit_grippers
from d1_control.utils.TrajectoryUtils import spline
from d1_control.utils.TransformHelpers import ep, eR

class Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str="d1_controller") -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.logger = self.get_logger()
        self.declare_ros_parameters()
        self.dt = self.get_timer_period_sec(TIMER_CTRL_PARAMETER_NAME)
        
        self.chain = KinematicChain(node=self, 
                                    baseframe=BASE_FRAME, 
                                    tipframe=TIP_FRAME, 
                                    expectedjointnames=JOINT_NAMES)
        self.q0 = None # initialize the starting joint positions

        # self.register_obk_subscription(
        #     SUB_VCMD_TOPIC,
        #     self.vcmd_callback, # type: ignore
        #     msg_type=VelocityCommand,
        # )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
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
        self._q = np.array(x_hat_msg.q_joints)[:NUM_JOINTS]
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
        self.pd = self.p0 # the desired tip position from the previous time step
        self.Rd = self.R0 # the desired tip orientation from the previous time step

        self.logger.info("Initialized inverse kinematics parameters.")

    def get_param(self, name: str) -> float:
        """Returns the double value associated with parameter `name`."""
        return self.get_parameter(name).get_parameter_value().double_value

    def get_timer_period_sec(self, name: str, key: str=TIMER_PERIOD_SEC_KEY) -> float:
        """Returns the timer_period_sec associated with parameter `name`."""
        string = self.get_parameter(name).get_parameter_value().string_value
        pairs = dict(pair.split(':') for pair in string.split(','))
        self.logger.info("pairs: %s" % pairs)
        dt = float(pairs[TIMER_PERIOD_SEC_KEY])
        return dt

    # def vcmd_callback(self, cmd_msg: VelocityCommand) -> None:
    #     """
    #     Update the commanded velocity of the gripper in the x and y direction. # TODO: add z direction.
    #     Update the commanded angular velocity about the z axis.

    #     Args:
    #         cmd_msg
    #     """
    #     v_x_min = self.get_param("v_x_min")
    #     v_x_max = self.get_param("v_x_max")
    #     self.vcmd[0] = min(max(cmd_msg.v_x, v_x_min), v_x_max)

    #     v_y_max = self.get_param("v_y_max")
    #     self.vcmd[1] = min(max(cmd_msg.v_y, -v_y_max), v_y_max)
        
    #     w_z_max = self.get_param("w_z_max")
    #     self.vcmd[2] = min(max(cmd_msg.w_z, -w_z_max), w_z_max)

    #     # self.logger.info("vcmd: %s" % self.vcmd)

    def compute_control(self) -> ObeliskControlMsg:
        """
        Compute the joint and gripper positions for the 6-DOF+1 robot. 
        
        joint1 to joint6 positions are in radians.
        gripper1 and gripper2 positions are in meters. 
        The gripper1 position is positive. 
        The gripper2 position is the negative of that of gripper1.
        The control message consists of eight inputs for Mujoco to simulate the 
        robot.
        
        Returns:
            obelisk_control_msg (ObeliskControlMsg): The control message.
        """
        t = self.t - self.start_time # seconds

        if self.q0 is None:
            return
        
        if t > INIT_TIME:
            return
        
        # Set the target tip position, velocity, and orientation to be achieved
        # after `INIT_TIME` has passed
        pg = np.array([0.2, 0.0, 0.3])
        vg = np.zeros(3)

        # Compute the desired position and velocity of the tip
        (pd, vd) = spline(t, INIT_TIME, self.p0, pg, self.v0, vg)
        Rd = self.R0
        wd = np.zeros(3)

        # IMPLEMENT THE 6 DOF INVERSE KINEMATICS
        # Grab the last joint values and desired tip position/orientation
        qdlast = self.qd
        pdlast = self.pd
        Rdlast = self.Rd

        (old_ptip, old_Rtip, Jv, Jw) = self.chain.fkin(qdlast)
        self.logger.info("old_ptip: %s" % old_ptip)
        J = np.concatenate((Jv, Jw))

        # Calculate errors
        error_pd = ep(pdlast, old_ptip)
        error_Rd = eR(Rdlast, old_Rtip)
        error = np.concatenate((error_pd, error_Rd))
        self.logger.info("error_pd: %s" % np.linalg.norm(error_pd))

        if np.linalg.norm(error) > MAX_ERROR_THRESHOLD:
            self.logger.error("Exceeded error threshold")
            return
        
        # Use J^-1 to calculate desired joint positions and velocities
        xddot = np.concatenate((vd, wd)) # desired task velocity
        xrdot = xddot + LAMBDA * error # reference task velocity
        J_winv = np.linalg.inv(J.T @ J + GAMMA ** 2 * np.identity(NUM_JOINTS)) @ J.T
        qddot = J_winv @ xrdot

        # Integrate the joint velocity
        qd = qdlast + self.dt * qddot
        self.logger.info("qddot: %s" % qddot)
        # Save the desired joint positions and tip position/orientation
        self.qd = qd
        self.pd = pd
        self.Rd = Rd

        self.logger.info("qd: %s" % qd)
        self.logger.info("pd: %s" % pd)
        # self.logger.info("Rd: %s" % Rd)

        # Set control inputs
        u_joints = qd.tolist()
        limit_joints(u_joints)
        u_grippers = [0.0, 0.0]
        u_joints.extend(u_grippers)

        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = u_joints
        position_setpoint_msg.q_des = u_joints
        self.obk_publishers[PUB_CONTROL_TOPIC].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # ignore type checking for now