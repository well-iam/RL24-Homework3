# Homework3

## :package: About

This package contains the developed code for the third homework of the Robotics Lab 2024/25 Course. The authors of the package are:
William Notaro, Chiara Panagrosso, Salvatore Piccolo, Roberto Rocco

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder (only works if the destination folder is empty), use:
```
$ git clone https://github.com/well-iam/RL24-Homework3 .
```
Alternatively, use:
```
$ git clone https://github.com/well-iam/RL24-Homework3
```

Build the two packages
```
$ colcon build
```
Source the setup files
```
$ source ~/ros2_ws/install/setup.bash
```

## :white_check_mark: Usage
- BLOB DETECTION MODE:  
To launch the Gazebo world with a specific world with the blue sphere inside, use this command:
```
 ros2 launch ros2_opencv sphere_detect.launch.py
```
This launch will automatically open an rqt_image_view where the detection result is shown. Move the blue sphere from Gazebo to see the updated blob detection on work.  
  
- POSITIONING TASK:  
Run the simulation environment through the launch file (by default the Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button):
```
ros2 launch iiwa_bringup iiwa.launch.py  use_vision:=true
```
This will launch a Gazebo world with a specific initial condition of the IIWA, and the ArUco Marker placed in front of it.
In this case, command_interface and robot_controller have been set in order to have a default speed interface and speed controller, while use_vision
guarantees that the camera will be included in the robot's model.
To run the controller, use the following code:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control 
```
Again, cmd_interface and traj_sel have been set by default in order to have velocity interface and a linear trajectory with cubic polynomial time law

Finally launch the node designated to recognize ArUco Marker from camera's point of view:
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201 reference_frame:=world
where:
- 'reference_frame' selects the reference compared to which the position of the aruco will be evaluated.
```
In this run, the terminal where you run the controller will require some offsets: these are supposed to be expressed in the ArUco frame. You can learn more about that 
in the documentation. By the way,  to test some valid trajectory insert [0, 0, 0.4] for position offset, while [3.14, 0, 1.57] for orientation ones.

- LOOK_AT_POINT TASK WITH VELOCITY CONTROLLER:  
Run the simulation environment through the launch file
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_vision:=true
```
This will launch a Gazebo world with a specific initial condition of the IIWA, and the ArUco Marker placed in front of it.
In this case, command_interface and robot_controller have been set in order to have speed interface and speed controller, while use_vision
guarantees that the camera will be included in the robot's model.
Then launch the node designated to recognize ArUco Marker from camera's point of view:
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
Now, the reference_frame is setted to camera_optical value: the position of the aruco will be evaluated with respect to a reference with a z axis lookin at the ArUco.
To run the controller, use the following code:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=velocity -p task:="look-at-point"
```
Again, cmd_interface and traj_sel have been set by default in order to have velocity interface and a linear trajectory with cubic polynomial time law
while the task flag allows to choose a different task to be performed. 




- LOOK_AT_POINT TASK WITH JOINT SPACE / OP. SPACE CONTROLLER:
Similarly to the previous case, run the simulation environment using the appropriate effort command interfaces:
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_vision:=true
```
To launch the node designated to recognize ArUco Marker from camera's point of view:
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
To launch the joint space controller use:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort_joint -p task:="look-at-point"
```
Instead, to launch the operational space controller:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort_operational -p task:="look-at-point"
```


- MERGE_CONTROLLERS MODE:
Run the simulation environment using the effort command interfaces:
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_vision:=true
```
To launch the node designated to recognize ArUco Marker from camera's point of view:
```
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
Then launch the merged controllers version using:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:=effort_operational -p task:="merge_controllers"
```

## :warning: Warning
Due to the nature of the project, you have to be quick when running the two parts. You have 5 seconds (a default timeout value difficult to change) to start the Gazebo program, connect the controller and play the simulation.   
We observed that starting the same project different times it led to different outcomes!  
In addition, we observed that the "look-at-point" mode didn't work when launched with rqt_image_view node.
