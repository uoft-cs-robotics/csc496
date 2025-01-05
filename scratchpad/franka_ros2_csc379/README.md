# franka_ros2_csc379

# Build

```bash
cd franka_ros2_csc379
source /opt/ros/humble/setup.bash
colcon build 
```

# Run franka ros2 cpp interfaces

```bash
source install/setup.bash

ros2 run franka_ros2_csc379 franka_state_publisher_ros2
# or
ros2 run franka_ros2_csc379 run_franka_impedance_control
# or
ros2 run franka_ros2_csc379 run_franka_impedance_control_ros2
# etc
```

# Run franka ros2 python clients

```bash
source /opt/ros/humble/setup.bash
cd franka_ros2_csc379/scripts
python3 <script>.py
```

# Linting the code (TA's only)
```bash
sudo apt-get install clang-format-11
./run_clang_format.sh
```
