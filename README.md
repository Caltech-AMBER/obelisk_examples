# obelisk-examples
A repo where we can hold examples using obelisk.

# Running the D1 Arm Python example
Make sure the XBox controller is plugged in; otherwise, it may not be detected
after the dev container is built.

Do initial setup of environment variables exposed to Docker:
```
bash setup.sh
```
Start the docker container:
```
cd docker

# No GPU
docker compose -f docker-compose-no-gpu.yml run --build obelisk_examples

# GPU
docker compose -f docker-compose.yml run --build obelisk_examples
```
Source base ROS, build Obelisk, activate Obelisk settings, and source Obelisk using one command:
```
obk
```
In the `obelisk` directory, run:
```
pip install -e $OBELISK_ROOT/obelisk/python
```

In the workspace (i.e. in `obelisk_examples`) build this package:
```
# build only 1 package
colcon build --symlink-install --parallel-workers $(nproc) --packages-select d1_control

# build all dummy packages
colcon build --symlink-install --parallel-workers $(nproc)
```
Source the generated bash script after building:
```
source install/setup.bash
```
Now we can launch the stack:
```
obk-launch config_file_path="${OBELISK_EXAMPLES_ROOT}/d1_control/configs/d1.yaml" device_name=onboard bag=false
```

# Setting up the Xbox remote
You can make sure that you can see the remote control with
```
sudo evtest
```

Then you can run
```
sudo chmod 666 /dev/input/eventX
```
where `X` is the number that you saw from evtest.

Then we can verify that ROS2 can see it with:
```
ros2 run joy joy_enumerate_devices
```

# Control
1. The arm initializes to a non-singular position
2. The XBox controller is used to command velocities (x, y, z) and angular 
velocities (wx, wy, wz) of the gripper. The buttons are used to open and close
the gripper. TODO: show a diagram of the XBox mapping.
3. Let's say the gripper is stationary. The controller takes in v_x = 0.1 m/s. 
The controller will create a spline to move to p_x = v_x * DT where DT is an 
appropriate scaling factor. The ending velocity should be v_x = 0 m/s. If
the user holds down on v_x = 0.1 m/s, the spline will always be updated.

Spline needs:
- current time TIME SINCE THE VELOCITY WAS INITIALLY COMMANDED
- period SET IT TO DT AKA THE SCALING FACTOR
- initial position (x, y, z) KNOWN
- target position (x, y, z) INITIAL POS + COMMANDED VELOCITY * DT
- initial velocity (x, y, z) GET LAST COMMANDED VELOCITY
- final velocity (x, y, z) LAST COMMANDED VELOCITY OR ZERO?

maybe the spline is really short, like 1/100th of a second
Spline needs:
- current time TIME SINCE THE VELOCITY WAS INITIALLY COMMANDED
- period SET IT TO DT AKA THE SCALING FACTOR
- initial position (x, y, z) KNOWN
- target position (x, y, z) INITIAL POS + INITIALLY COMMANDED VELOCITY * DT
- initial velocity (x, y, z) INITIALLY COMMANDED VELOCITY
- final velocity (x, y, z) MOST RECENTLY COMMANDED VELOCITY

Spline is recomputed at like 1/100th of a second
-------

Easier thing
- User specifies a point in 3D space
- Also orientation?
- Arm goes to point and orientation
- Just need Newton Raphson

this doesn't use rotation matrix though... maybe worry about this later?

In ME 134
- Real time inverse kinematics. Given tip position at every time step.
self.chain.fkin(). at every iteration <= better
- Newton Raphson: Figure out how to go from one joint position to another
when given (x, y, z) for the tip. Iterative

Either way we need the Kinematic Chain