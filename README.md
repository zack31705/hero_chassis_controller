# hero_chassis_controller

## Overview

This is the final assignment project for the RoboMaster / ros_control course. 
It implements a complete `ros_control` hardware interface plugin for a four-wheeled Mecanum robot (Hero chassis).

The controller implements the following core functionalities:
- **Four independent PID controllers** for precise wheel velocity control (dynamically configurable via `rqt_reconfigure`).
- **Inverse Kinematics**: Translates `geometry_msgs/Twist` (`/cmd_vel`) commands into individual wheel target velocities.
- **Forward Kinematics & Odometry**: Calculates the chassis velocity from actual wheel velocities, integrates it to track the robot's pose (X, Y, theta), and publishes `nav_msgs/Odometry` and `TF` transforms (`odom` to `base_link`).
- **Global Coordinate Velocity Mode**: Uses TF to transform global velocity commands into the robot's local coordinate frame, allowing the robot to translate in absolute directions regardless of its heading.
- **Acceleration Limiting**: Smooths out velocity commands to prevent sudden jerks and wheel slippage.

**Keywords:** RoboMaster, ROS, ros_control, Mecanum Wheel, Kinematics, PID

## License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author:** Cai Zekai <br />
**Affiliation:** [Dynamicx]()<br />
**Maintainer:** Cai Zekai, czk3091084877@qq.com

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description_for_task](https://github.com/gdut-dynamic-x/rm_description) (Robot URDF models),
- `controller_interface`, `hardware_interface`, `pluginlib`
- `control_toolbox`, `realtime_tools`
- `geometry_msgs`, `nav_msgs`, `tf`

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using:
```bash
cd catkin_workspace/src
git clone https://github.com/zack31705/hero_chassis_controller.git
cd ../
rosdep install --from-paths . --ignore-src
catkin build hero_chassis_controller
```

## Usage

Run the simulation and controller with:
```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
```

To control the robot via keyboard, open a new terminal and run:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

To dynamically tune PID parameters, run:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Config files

**controllers.yaml**: Contains the configuration for `hero_chassis_controller` and `joint_state_controller`.
- Includes PID parameters (p, i, d, i_clamp) for all four wheels.

- Includes a flag `use_global_frame` to switch between local chassis velocity mode and global odom velocity mode.

## Launch files

**run_simulation_and_controller.launch**: Launches the Gazebo simulation with the Hero chassis, loads the `controllers.yaml`, and spawns the `hero_chassis_controller` and `joint_state_controller`.

## Implementation Details (For Assignment Evaluation)

1. **PID Control**: Implemented using `control_toolbox::Pid`. Four independent instances handle each wheel. Parameters are loaded dynamically via `NodeHandle`.
2. **Kinematics**: Inverse kinematics maps \(v_x, v_y, w_z\) to wheel angular velocities. Forward kinematics reconstructs base velocity from encoder (simulated) feedback.
3. **Odometry**: Utilizes a `tf::TransformBroadcaster` to publish `odom` -> `base_link`, and publishes `nav_msgs/Odometry` to the `/odom` topic.
4. **Global Velocity Transformation**: Queries `tf::TransformListener` to get the current Yaw angle, then uses a rotation matrix (or basis projection) to transform global \(v_x, v_y\) to local chassis velocities.
5. **Extra Feature**: The controller supports generating a **heart-shaped trajectory** via hardcoded time state-machine logic for demonstration purposes (can be toggled in code).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/zack31705/hero_chassis_controller.git).