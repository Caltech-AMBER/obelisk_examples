# obelisk-examples
A repo where we can hold examples using obelisk.

# Running the D1 Arm Python example
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
