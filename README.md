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
