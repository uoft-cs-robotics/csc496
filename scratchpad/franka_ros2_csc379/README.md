# franka_ros2_csc379

Starter code for franka ros2 csc379

# Build

```bash
cd franka_ros2_csc379 # or scratchpad
source /opt/ros/humble/setup.bash
colcon build 
```

# Run cpp interfaces

```bash
source install/setup.bash

ros2 run franka_ros2_csc379 franka_state_publisher
# or
ros2 run franka_ros2_csc379 franka_impedance_control
# etc
```

# Run python clients

```bash
source /opt/ros/humble/setup.bash
cd scripts
python3 <script>.py
```

# Linting the code (TA's only)
```bash
sudo apt-get install clang-format-11
./run_clang_format.sh
```
