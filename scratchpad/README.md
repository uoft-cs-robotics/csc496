
# csc379 scratchpad

Starter code for csc379

# Build

```bash
cd scratchpad
source /opt/ros/humble/setup.bash
colcon build 
```

# Run franka ros2 cpp interfaces

```bash
source install/setup.bash

ros2 run franka_ros2_csc379 franka_state_publisher
# or
ros2 run franka_ros2_csc379 franka_impedance_control
# etc
```

# Run franka ros2 python clients

```bash
source /opt/ros/humble/setup.bash
cd franka_ros2_csc379/scripts
python3 <script>.py
```
