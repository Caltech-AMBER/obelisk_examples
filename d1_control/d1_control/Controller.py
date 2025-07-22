from typing import Optional

import numpy as np
from numpy.linalg import norm, solve
from math import pi
import pinocchio as pin

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

from obelisk_control_msgs.msg import PositionSetpoint, VelocityCommand
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

from d1_control.utils.constants import *
from d1_control.utils.enums import Mode

from d1_control.utils.joystick_enums import Axis, Button
from d1_control.utils.joystick import Joystick

from d1_control.utils.ControlUtils import limit_joints, limit_gripper
from d1_control.utils.TrajectoryUtils import goto, spline
from d1_control.utils.TransformHelpers import p_from_Point, R_from_Quaternion

class Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str) -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.get_logger().info("Initializing controller")
        self.info = self.get_logger().info
        self.error = self.get_logger().error

        # Load the urdf model
        self.model = pin.buildModelFromUrdf(URDF_FILENAME)
        # Create data required by algorithms
        self.data = self.model.createData()

        self.mode = None
        self.mode_start_time = None

        self.q0 = None # initialize the starting joint positions
        self._qd = None # initialize the desired joint positions
        
        self.joy = Joystick() # Initialize the joystick

        self.register_obk_subscription(
            SUB_GOAL_NAME,
            self.goal_callback, # type: ignore
            msg_type=PoseStamped,
        )
        
        self.register_obk_subscription(
            SUB_JOY_NAME,
            self.joy_callback, # type: ignore
            msg_type=Joy,
        )
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.declare_ros_parameters()
        self.dt = self.get_timer_period_sec(TIMER_CTRL_NAME)
        self.moving_time = self.dt
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg: The Obelisk message containing the 
            state estimate of the eight joints representing the arm.
        """
        if len(x_hat_msg.q_joints) != NUM_CONTROL_INPUTS: # this may occur when simulating the robot
            return
        
        # Update the state
        servo_state = np.array(x_hat_msg.q_joints)
        self._q = servo_state[:NUM_JOINTS]
        self._gripper = servo_state[-2] # gripper position is positive

        # Initialize kinematic parameters
        if self.q0 is None:
            self.init_kinematic_parameters(self._q, self._gripper)

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

    @property
    def qd(self) -> np.ndarray:
        """The desired joint positions (radians)."""
        return self._qd
    
    @qd.setter
    def qd(self, value: np.ndarray) -> None:
        if value is not None:
            limit_joints(value)
        self._qd = value

    @property
    def gripperd(self) -> float:
        """
        The desired gripper position (meters) i.e. the distance between 
        the center of the clamps and the inner face of a clamp.
        """
        return self._gripperd
    
    @gripperd.setter
    def gripperd(self, value: float) -> None:
        if value is not None:
            value = limit_gripper(value)
        self._gripperd = value

    @property
    def gripperg(self) -> float:
        """
        The goal gripper position (meters) i.e. the distance between 
        the center of the clamps and the inner face of a clamp.
        """
        return self._gripperg
    
    @gripperg.setter
    def gripperg(self, value: float) -> None:
        if value is not None:
            value = limit_gripper(value)
        self._gripperg = value

    @property
    def control_inputs(self) -> np.ndarray:
        """
        The control inputs for the Mujoco simulator and computing inverse
        kinematics.
        """
        return np.hstack((self._qd, self._gripperd, -self._gripperd))

    @control_inputs.setter
    def control_inputs(self, value: np.ndarray) -> None:
        """
        Sets the private variables for computing the control input.

        Args:
            value ((8,) np.ndarray): first six values are the joint positions.
            Last two values are the gripper positions.
        """
        self.qd = value[:NUM_JOINTS]
        self.gripperd = value[-2]

    @property
    def mode(self) -> Mode:
        return self._mode
    
    @mode.setter
    def mode(self, value: Mode) -> None:
        """Sets the mode and resets the mode start time."""
        if value == Mode.INIT:
            self.q0 = self.qd
        self._mode = value
        self.reset_mode_start_time()

    def reset_mode_start_time(self) -> None:
        """
        Sets the mode start time to the current time.
        """
        self.mode_start_time = self.get_clock().now().nanoseconds * 1e-9

    def init_kinematic_parameters(self, q0: np.ndarray, gripper0: float) -> None:
        """
        Initialize parameters for computing the kinematics of
        the arm.
        
        Initialize position of gripper.

        Args:
            q0 ((6,)-shape np.ndarray): initial joint positions
            gripper0 (float): initial gripper position
        """
        self.q0 = q0 # the initial joint positions
        self.qd = q0 # the desired joint positions
        self.gripperd = gripper0 # the desired gripper position

        self.info("Control inputs: %s" % self.control_inputs)
        self.mode = Mode.INIT
    
    def init_inverse_kinematic_parameters(self) -> None:
        """
        Initialize parameters for computing the inverse kinematics of
        the arm.
        """
        # Initialize the desired position, velocity, and orientation
        pin.forwardKinematics(self.model, self.data, self.control_inputs)
        posed = self.data.oMi[JOINT_ID]
        self.pd = posed.translation.T # the desired tip position
        self.vd = np.zeros(3) # the desired tip velocity
        self.Rd = posed.rotation # the desired tip orientation
        self.wd = np.zeros(3) # the desired tip angular velocities
        
        self.info("pd: %r" % self.pd)
        self.info("Rd: %r" % self.Rd)
        
    def get_param(self, name: str) -> float:
        """Returns the double value associated with parameter `name`."""
        return self.get_parameter(name).get_parameter_value().double_value

    def get_timer_period_sec(self, name: str, key: str=TIMER_PERIOD_SEC_KEY) -> float:
        """
        Returns the `timer_period_sec` associated with parameter `name`.

        The `timer_period_sec` is the duration in seconds between consecutive
        invocations of the timer with the parameter `name`. 
        """
        string = self.get_parameter(name).get_parameter_value().string_value
        pairs = dict(pair.split(':') for pair in string.split(','))
        dt = float(pairs[TIMER_PERIOD_SEC_KEY])
        return dt

    def joy_callback(self, msg: Joy) -> None:
        """
        Called when a message from the joystick is received.

        Args:
            msg: contains the joystick readings (range: [-1, 1])
        """
        if self.mode in [None, Mode.INIT]: # , Mode.MOVING]:
            return
        
        # GET VELOCITY COMMANDS IN WORLD FRAME
        axes = msg.axes
        buttons = msg.buttons

        # Set tip linear velocity
        self.joy.vx = -axes[Axis.LEFT_X] # Positive if stick is pushed rightward
        self.joy.vy = axes[Axis.LEFT_Y] # Positive if stick is pushed upward
        self.joy.vz = axes[Axis.DPAD_Y] * self.joy.speed # Positive if DPAD_UP is pushed

        # Set gripper velocity
        self.joy.vgripper = -axes[Axis.DPAD_X] # Positive if stick is pushed rightward

        # Set tip angular velocity about the x-axis and y-axis
        self.joy.wx = -axes[Axis.RIGHT_X] # Positive if stick is pushed rightward
        self.joy.wy = axes[Axis.RIGHT_Y] # Positive if stick is pushed upward
        
        # Set tip angular velocity about the z-axis
        self.joy.wz = 0
        if buttons[Button.B]:
            self.joy.wz = self.joy.speed
        if buttons[Button.X]:
            self.joy.wz = -self.joy.speed

        # Change joystick speeds
        if buttons[Button.Y]:
            self.joy.speed += JOY_SPEED_INCREMENT
        if buttons[Button.A]:
            self.joy.speed -= JOY_SPEED_INCREMENT
        # self.info("joy speed: %s" % self.joy.speed)

        # Reinitialize the robot
        if buttons[Button.START]:
            self.mode = Mode.INIT
            self.info("Reinitializing robot")
            return

        # COMPUTE GOAL TIP POSITION/ORIENTATION BASED OFF THE VELOCITY COMMANDS
        # Set the initial conditions to the current desired values
        self.p0 = self.pd
        self.v0 = self.vd
        self.R0 = self.Rd
        self.w0 = self.wd

        # Set velocity commands
        v_cmd = self.joy.get_v_cmd() * V_MAX
        w_cmd = self.joy.get_w_cmd() * W_MAX
        vgripper_cmd = self.joy.vgripper

        self.info("v_cmd: %s" % self.joy.get_v_cmd())
        self.info("w_cmd: %s" % self.joy.get_w_cmd())
        self.info("vgripper_cmd: %s" % self.joy.vgripper)

        # Halt the robot if velocities are commanded to be zero
        if not (norm(v_cmd) or norm(w_cmd) or vgripper_cmd):
            self.vd = np.zeros(3)
            self.wd = np.zeros(3)
            self.mode = Mode.WAITING
            return

        # Set the goal tip velocities
        self.vg = v_cmd
        self.wg = w_cmd

        # Set the goal tip position
        self.pg = self.p0 + v_cmd * self.dt
        
        # Set the goal tip orientation
        if norm(w_cmd) > 1e-6: # avoid division by zero
            # Compute the rotation increment using the exponential map
            delta_R = pin.exp3(w_cmd * self.dt) # rotation increment
            self.Rg = self.R0 @ delta_R
        else:
            self.Rg = self.R0 # No rotation if angular velocity is negligible   

        # Set the goal gripper position
        self.gripperg = self.gripperd + vgripper_cmd * self.dt

        # Set the moving time
        # self.moving_time = JOY_MOVING_TIME # FIXME: looks smoother in sim for some reason
        self.moving_time = self.dt

        # Set mode
        self.mode = Mode.MOVING

        # self.info("Goal position: %s, Goal orientation: %s" % (self.pg, self.Rg))
        # self.info("Initial position: %s" % self.p0)
        # self.info("Desired position: %s" % self.pd)

        # TODO: figure out why gripper goes in wrong direction sometimes.
        # Seems like p0, pd, pg, v0, vd, and vg are correct. So maybe there's
        # something wrong with the code on the arm.

    # def v_cmd_callback(self, cmd_msg: VelocityCommand) -> None:
    #     """
    #     Update the commanded velocity of the gripper in the x and y direction.
    #     Update the commanded angular velocity about the z axis.

    #     Args:
    #         cmd_msg
    #     """
    #     v_x_min = self.get_param("v_x_min")
    #     v_x_max = self.get_param("v_x_max")
    #     self.v_cmd[0] = min(max(cmd_msg.v_x, v_x_min), v_x_max)

    #     v_y_max = self.get_param("v_y_max")
    #     self.v_cmd[1] = min(max(cmd_msg.v_y, -v_y_max), v_y_max)
        
    #     w_z_max = self.get_param("w_z_max")
    #     self.v_cmd[2] = min(max(cmd_msg.w_z, -w_z_max), w_z_max)

    #     self.info("v_cmd: %s" % self.v_cmd)

    def compute_control(self) -> Optional[ObeliskControlMsg]:
        """
        Compute the joint and gripper positions for the 6-DOF+1 robot. 
        
        joint1 to joint6 positions are in radians.
        gripper1 and gripper2 positions are in meters. 
        The gripper1 position is positive. 
        The gripper2 position is the negative of that of gripper1.
        The control message consists of eight inputs for Mujoco to simulate the 
        robot.
        
        Returns:
            obelisk_control_msg: The control message.
        """
        if self.mode_start_time is None or self.q0 is None:
            return
        
        t = self.t - self.mode_start_time # seconds

        match self.mode:
            case Mode.INIT:
                # Compute the desired joint positions
                (qd, _) = goto(t, INIT_TIME, self.q0, QG_INIT)
                self.qd = qd
                self.gripperd = GRIPPERG_INIT
                if t + self.dt > INIT_TIME:
                    self.init_inverse_kinematic_parameters()
                    self.mode = Mode.WAITING
            case Mode.MOVING:
                # TODO: incorporate orientation
                control_inputs = self.inverse_kinematics(
                    t, 
                    self.moving_time, 
                    self.p0, 
                    self.pg, 
                    self.v0, 
                    self.vg
                )
                # if control_inputs is None: # Occurs if we have an error threshold
                #     self.error("Failed to compute inverse kinematics.")
                #     return
                self.control_inputs = control_inputs
                self.gripperd = self.gripperg
                # self.info("Moving control inputs: %s" % self.control_inputs)
                # self.info("t: %f, self.moving_time: %f" % (t, self.moving_time))

                # self.info("p0: %s" % self.p0[0])
                # self.info("pd: %s" % self.pd[0])
                # self.info("pg: %s\n" % self.pg[0])
                
                # self.info("v0: %s" % self.v0[0])
                # self.info("vd: %s" % self.vd[0])
                # self.info("vg: %s\n\n" % self.vg[0])

                # FIXME: Uncomment later?
                if t + self.dt > self.moving_time:
                    self.mode = Mode.WAITING
            case Mode.WAITING:
                # self.info("Waiting for next command.")
                return
            case _:
                self.error("Unknown mode.")
                return

        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = self.control_inputs.tolist()
        position_setpoint_msg.q_des = self.control_inputs.tolist()
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
            vg: np.ndarray,
            # R0: np.ndarray,
            # Rg: np.ndarray,
            # w0: np.ndarray,
            # wg: np.ndarray
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
            # R0 ((3,3)-shape np.ndarray): initial tip orientation
            # Rg ((3,3)-shape np.ndarray): goal tip orientation
            # w0 ((3,)-shape np.ndarray): initial tip angular velocity (rad/s)
            # wg ((3,)-shape np.ndarray): goal tip angular velocity (rad/s)

        Returns:
            control_inputs ((7,)-shape np.ndarray): desired control inputs
        """
        # Compute the desired position and velocity of the tip
        (pd, vd) = spline(t, moving_time, p0, pg, v0, vg)
        Rd = self.Rd
        wd = np.zeros(3)
        desired_pose = pin.SE3(Rd, pd)

        # Grab the last joint values and desired tip position/orientation
        control_inputs_last = self.control_inputs
        # pdlast = self.pd
        # Rdlast = self.Rd

        # Perform forward kinematics over the kinematic tree
        pin.forwardKinematics(self.model, self.data, control_inputs_last)
        current2desired = self.data.oMi[JOINT_ID].actInv(desired_pose) # in joint frame (i.e. relative to current pose)
        error = pin.log(current2desired).vector # error between desired pose and current pose in joint frame
        J = pin.computeJointJacobian(self.model, self.data, control_inputs_last, JOINT_ID) # in joint frame
        J = -np.dot(pin.Jlog6(current2desired.inverse()), J) # in the appropriate frame
        # use damped pseudoinverse to avoid problems at singularities
        v = -J.T.dot(solve(J.dot(J.T) + DAMPING_FACTOR * np.eye(NUM_JOINTS), error))
        control_inputs = pin.integrate(self.model, control_inputs_last, v * self.dt)

        # Save the desired control inputs
        # self.control_inputs = control_inputs # necessary for iterative Newton-Raphson

        # Save the tip position/velocity/orientation
        self.pd = pd
        self.vd = vd
        self.Rd = Rd
        self.wd = wd

        # self.info("control_inputs: %s" % control_inputs)
        # self.info("time: %f" % t)
        # self.info("pd: %s" % pd)
        # self.info("Rd: %s" % Rd)

        return control_inputs
    
    def goal_callback(self, msg: PoseStamped) -> None:
        """Called when we manually publish a goal pose from the command-line."""
        # TODO: If this is called while the mode is moving, then the spline
        # may need to be recomputed.
        # Set the starting position, velocity, and orientation.
        # Use the desired quantities because they are less noisy than the
        # actual quantities.
        if self.mode in [None, Mode.INIT]:
            return
        
        # Set initial conditions
        self.p0 = self.pd
        self.v0 = self.vd
        self.R0 = self.Rd
        self.w0 = self.wd

        # Set the goal
        pose = msg.pose
        point = pose.position
        quat = pose.orientation
        self.pg = p_from_Point(point) # goal tip position (meters)
        self.vg = np.zeros(3) # goal tip velocity
        self.Rg = R_from_Quaternion(quat) # goal orientation
        self.wg = np.zeros(3) # goal angular velocity (rad/s)

        self.gripperg = self.gripperd # FIXME: goal gripper position (meters)
        self.moving_time = init_kinematic_parameters # the number of seconds it will take to reach the goal
        self.mode = Mode.MOVING

    