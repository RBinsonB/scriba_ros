Scriba robot base ROS packages:
- **scriba** Scriba robot main package

- **scriba_description** Robot description and URDF files.

- **scriba_msgs** Message repository.

For more detailed information on the robot, please take a look at [the robot repository](https://github.com/RBinsonB/Scriba_robot). For localization related packages, please take a look at the [scriba_localization package](https://github.com/RBinsonB/scriba_localization)

:bangbang: Code is relatively old and could be cleaned up and improved.

:bangbang: Services don't work on Arduino with the last rosserial_python and rosserial_arduino packages (0.8.0). Building an older version from source is required in order to use services. Older packages are included in the repository and can be built after uninstalling the repo packages.

:bangbang: There is a bug in the lastest ROS versions (Melodic and Lunar) preventing the correct display on robot models in RViz if you have a local setting other than US. A quick fix working for me was to type the following command before launching RViz:
```
export LC_NUMERIC="en_US.UTF-8"
```

# scriba

## Overview

Includes Scriba robot main nodes and launch files.

## Launch files
- **scriba_bringup.launch** Launches all required components for basic odometry and motor command.

- **scriba_odom.launch** Launches all required components for basic odometry.

- **scriba_rviz_remote.launch** Launches RViz with all required parameters for a remote display of the odometry

## Nodes

### scriba_base_controller

Converts velocity commands (vel_cmd) to custom motor command (mot_cmd) to be sent to the Arduino of the drive unit.

#### Subscribed topics
- `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
    
    Velocity command to drive the robot.

#### Published topics
- `mot_cmd` ([scriba_msgs/mot_cmd](/scriba_msgs/msg/mot_cmd.msg))
	
	Front wheel steer angle and motor command send to the traction unit Arduino controller.

### scriba_odometry_broadcaster
Receives odometry data from the custom data_odom message type and computes the odometry accordingly. The node integrates a Kalman filter to estimate the steer angle from the stepper motor command and the potentiometer readings.

#### Subscribed topics
- `odom_data` ([scriba_msgs/odom_data](/scriba_msgs/msg/odom_data.msg))
    
    Data from the traction unit Arduino needed to compute the odometry.

#### Published topics

- `odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
    
    Robot odometry.

- `odometry_motion` ([scriba_msgs/motion_odom](/scriba_msgs/msg/motion_odom.msg))
    
    Odometry motion vector. Published at every loop iteration and used by the position EKF.

- `joint_states` ([sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html))
    
    Robot joint state from the odometry.

## Scripts
- **front_wheel_calibration.py** Script for calibration of the front wheel angle. Uses a custom front_wheel_calibration service type.

# scriba_description
Includes Scriba URDF model and launch files for display in RViz.

## Launch files

- **scriba_rviz.launch** Displays the URDF robot model in Rviz with a GUI to configure joint values.

# scriba_msgs
Includes Scriba custom messages.

## Message files
- **data_odom.msg** Custom message for odometry data sent by the Arduino of the drive unit.

- **mot_cmd.msg** Custom message for the motor command sent to the Arduino of the drive unit.

- **motion_odom.msg** Odometry vector including front wheel steer angle and front wheel traveled distance

- **localization_fix.msg** 2D pose localization with covariance (x, y, yaw angle)

## Service files

- **front_wheel_calibration.srv** Custom service for calibration of the front wheel angle. Service server hosted by the Arduino of the drive unit.


