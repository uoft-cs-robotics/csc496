# slicer_ros2_ws

csc379's slicer and ros2 workspace

# Build

Slicer should already have been built by the docker, we build the packages in
this folder by:

```bash
./colcon_build.sh
```

# Run 3D Slicer and ROS2 nodes

## In one terminal, run 3D Slicer:

```bash
slicer
```

### Note

The first time you run Slicer, you need to add the modules directories:
- slicer_ros2_ws/build/slicer_ros2_module/lib/Slicer-5.6/qt-loadable-modules
- slicer_ros2_ws/build/slicer_ros2_module/lib/Slicer-5.6/qt-scripted-modules
in the Application Settings (Edit > Application Settings > Modules > Additional module paths > Add). Then restart Slicer.

## In another terminal, launch the franka_bringup node:

```bash
source install/setup.bash
ros2 launch franka_bringup franka.launch.py
```

## (Optional) To test if robot visual is moving, in another terminal, run the joint_state_publisher node:

```bash
source install/setup.bash
ros2 run franka_bringup fake_joints
```

## In 3D Slicer

Load the ROS2 module (Menubar > Modules search icon > Search for "ROS2" and select it).

You should see the following:

![Add new robot](./images/add_robot.png)

Click "+ Add new robot". You should see the following:

![Load robot](./images/load_robot.png)

Click "Load robot" and you should see the robot model in the 3D view.


