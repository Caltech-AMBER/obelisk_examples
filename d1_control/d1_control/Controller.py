from typing import Optional

import numpy as np
from numpy.linalg import norm, solve
from math import pi
import pinocchio as pin

from obelisk_control_msgs.msg import PositionSetpoint, VelocityCommand
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn, LifecycleNode

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

from d1_control.constants import *

from d1_control.utils.ControlUtils import limit_joints, limit_grippers
from d1_control.utils.TrajectoryUtils import goto, spline
from d1_control.utils.TransformHelpers import p_from_Point, R_from_Quaternion
from d1_control.utils.enums import Mode

from geometry_msgs.msg import PoseStamped

class Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str) -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.get_logger().info("Initializing controller")
        self.info = self.get_logger().info

        self.info("Loading urdf model")
        # Load the urdf model
        self.model = pin.buildModelFromUrdf(URDF_FILENAME)
        # Create data required by algorithms
        self.data = self.model.createData()

        self.mode = None
        self.mode_start_time = None
        self.q0 = None # initialize the starting joint positions
        self.gripper0 = None # initialize the starting gripper positions

        self.register_obk_subscription(
            SUB_GOAL_NAME,
            self.goal_callback, # type: ignore
            msg_type=PoseStamped,
        )
        self.info("Finished initializing")
        self.info("is lifecycle node: %s" % isinstance(self, LifecycleNode))

        # self.register_obk_subscription(
        #     SUB_VCMD_NAME,
        #     self.vcmd_callback, # type: ignore
        #     msg_type=VelocityCommand,
        # )
        

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        self.info("Configuring controller")
        super().on_configure(state)
        self.start_time = self.get_clock().now() # FIXME: DELETE
        self.info("Configuring")
        # Set the goal
        self.pg = np.array([0.2, 0.0, 0.3]) # goal tip position (meters)
        self.vg = np.zeros(3) # goal tip velocity
        self.Rg = None # goal orientation
        self.wg = np.zeros(3) # goal angular velocity (rad/s)
        self.gripper_g = 0.02 # goal gripper position (meters)
        self.moving_time = 5 # the number of seconds it will take to reach the goal
        self.info("Configuring")
        self.declare_ros_parameters()
        self.info("Configuring")
        self.dt = self.get_timer_period_sec(TIMER_CTRL_NAME)
        self.info("Configured controller")
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg (ObeliskEstimatorMsg): The Obelisk message containing the 
            state estimate of the eight joints representing the arm.
        """
        servo_state = np.array(x_hat_msg.q_joints)
        self._q = servo_state[:NUM_JOINTS]
        self._gripper = np.array([servo_state[-1], -servo_state[-1]])
        if self.q0 is None:
            self.init_kinematics_parameters(self._q, self._gripper)

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

    def init_kinematics_parameters(self, q0: np.ndarray, gripper0: np.ndarray) -> None:
        """
        Initialize parameters for computing the kinematics of
        the arm.
        
        Initialize position of gripper. Confirm that the tip position
        is in between the gripper.
        """
        self.q0 = q0
        self.gripper0 = gripper0

        control_inputs = np.hstack((q0, gripper0))
        self.info("Control inputs: %s" % control_inputs)

        # Initialize the desired position, velocity, and orientation
        pin.forwardKinematics(self.model, self.data, control_inputs)
        pose0 = self.data.oMi[JOINT_ID]
        self.p0 = pose0.translation.T
        self.R0 = pose0.rotation
        self.v0 = np.zeros(3)

        self.info("p0: %r" % self.p0)
        self.info("R0: %r" % self.R0)
        
        self.qd = self.q0 # the desired joint positions from the previous time step
        self.pd = self.p0 # the desired tip position from the previous time step
        self.Rd = self.R0 # the desired tip orientation from the previous time step

        self.mode = Mode.INIT
        self.reset_mode_start_time()
        self.info("Initialized inverse kinematics parameters.")

    def get_param(self, name: str) -> float:
        """Returns the double value associated with parameter `name`."""
        return self.get_parameter(name).get_parameter_value().double_value

    def get_timer_period_sec(self, name: str, key: str=TIMER_PERIOD_SEC_KEY) -> float:
        """Returns the timer_period_sec associated with parameter `name`."""
        string = self.get_parameter(name).get_parameter_value().string_value
        pairs = dict(pair.split(':') for pair in string.split(','))
        self.info("pairs: %s" % pairs)
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

    #     # self.info("vcmd: %s" % self.vcmd)

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
        if self.mode_start_time is None or self.q0 is None:
            return
        
        t = self.t - self.mode_start_time # seconds

        # self.info("Mode: %s" % self.mode)
        match self.mode:
            case Mode.INIT:
                # Compute the desired joint position of the tip
                (qd, _) = goto(t, INIT_TIME, self.q0, Q_INIT)
                if t + self.dt > INIT_TIME:
                    self.reset_mode_start_time()
                    self.mode = Mode.WAITING
            case Mode.MOVING:
                # TODO: incorporate orientation
                qd = self.inverse_kinematics(t, self.moving_time, self.p0, self.pg, self.v0, self.vg)
                if qd is None:
                    return
                if t + self.dt > self.moving_time:
                    self.reset_mode_start_time()
                    self.mode = Mode.WAITING
            case Mode.WAITING:
                # self.info("Waiting for next command.")
                return
            case _:
                self.logger.error("Unknown mode.")
                return
    
        # Set control inputs
        u_joints = qd.tolist()
        limit_joints(u_joints)
        u_grippers = [0.0, 0.0]
        u_joints.extend(u_grippers)

        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = u_joints
        position_setpoint_msg.q_des = u_joints
        self.obk_publishers[PUB_CONTROL_NAME].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # ignore type checking for now
    
    def inverse_kinematics(
            self, 
            t: float, 
            moving_time: float, 
            p0: np.ndarray,
            pg: np.ndarray,
            v0: np.ndarray,
            vg: np.ndarray
        ) -> Optional[np.ndarray]:
        """
        Implement the 6 DOF Inverse Kinematics.

        Args:
            t (float): time (seconds) since the goal was requested
            moving_time (float): time (seconds) to achieve the goal
            p0 ((3,)-shape np.ndarray): initial tip position (meters)
            pg ((3,)-shape np.ndarray): goal tip position (meters)
            v0 ((3,)-shape np.ndarray): initial tip velocity (m/s)
            vg ((3,)-shape np.ndarray): goal tip velocity (m/s)

        Returns:
            qd ((6,)-shape np.ndarray): desired joint position
        """
        # Compute the desired position and velocity of the tip
        (pd, vd) = spline(t, moving_time, p0, pg, v0, vg)
        Rd = self.R0
        wd = np.zeros(3)
        desired_pose = pin.SE3(Rd, pd)

        # Grab the last joint values and desired tip position/orientation
        qdlast = self.qd
        # pdlast = self.pd
        # Rdlast = self.Rd

        # Perform forward kinematics over the kinematic tree
        pin.forwardKinematics(self.model, self.data, qdlast)
        current2desired = self.data.oMi[JOINT_ID].actInv(desired_pose) # in joint frame (i.e. relative to current pose)
        error = pin.log(current2desired).vector # error between desired pose and current pose in joint frame
        J = pin.computeJointJacobian(self.model, self.data, qdlast, JOINT_ID) # in joint frame
        J = -np.dot(pin.Jlog6(current2desired.inverse()), J) # in the appropriate frame
        # use damped pseudoinverse to avoid problems at singularities
        v = -J.T.dot(solve(J.dot(J.T) + DAMPING_FACTOR * np.eye(NUM_JOINTS), error))
        qd = pin.integrate(self.model, qdlast, v * self.dt)

        # Save the desired joint positions and tip position/orientation
        self.qd = qd
        self.pd = pd
        self.Rd = Rd

        self.info("qd: %s" % qd)
        self.info("pd: %s" % pd)
        # self.info("Rd: %s" % Rd)

        """
        # Compute the new pose of the tip
        pin.forwardKinematics(self.model, self.data, qd)

        # Update the current velocity and angular velocity of the tip
        self._v = ep(new_ptip, old_ptip) / self.dt
        self._w = eR(new_Rtip, old_Rtip) / self.dt

        # Update the current position and orientation of the tip
        self._p = new_ptip
        self._R = new_Rtip
        """
        return qd
    
    def goal_callback(self, msg: PoseStamped) -> None:
        """Called when we manually publish a goal pose from the command-line."""
        # Set the starting position, velocity, and orientation
        self.p0 = self._p
        self.v0 = self._v
        self.R0 = self._R
        self.w0 = self._w

        # Set the goal position, velocity, and orientation
        point = msg.position
        quat = msg.orientation
        self.pg = p_from_Point(point)
        self.vg = np.zeros(3)
        self.Rg = R_from_Quaternion(quat)
        self.wg = np.zeros(3)

        self.mode = Mode.MOVING

    def reset_mode_start_time(self):
        """
        Sets the mode start time to the current time.
        """
        self.mode_start_time = self.get_clock().now().nanoseconds * 1e-9