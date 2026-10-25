# design_of_mechatronic_systems_lab
Name: Ahmed Khaled  ID:9275
Name:Badr Hasan Fouad ID:9805
Name:Abanoub Micheal ID:9236

 Packages structure:
 ros2_lab_1_ws
│
└── src/
    │
    ├── turtle_interface/            # Custom ROS 2 interfaces
    │   ├── msg/
    │   │   └── TargetPose.msg
    │   ├── action/
    │   │   ├── GoToPose.action
    │   │   └── DrawSquare.action
    │   └── srv/
    │       └── SetSpeedLimits.srv
    │
    └── turtle_control/             # Python implementation
        ├── turtle_control/
        │   ├── go_to_pose_server.py
        │   ├── reset_server.py
        │   ├── set_speed_server.py
        │   └── square_action_server.py
        ├── launch/
        ├── config/
        ├── setup.py
        ├── package.xml
        └── setup.cfg

Build Steps:
TO USE GO TO POSE
terminal 1:
cd ~/ros2_lab_1_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 run turtlesim turtlesim_node

terminal 2:
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtle_control --symlink-install
source install/setup.bash
 ros2 interface list
 ros2 run turtle_control go_to_pose_server

 terminal 3:
 source /opt/ros/jazzy/setup.bash
 colcon build --packages-select turtle_interface --symlink-install
 source install/setup.bash
 ros2 action list -t
 ros2 action send_goal /go_to_pose turtle_interface/action/GoToPose "{target:{x:2.0,y:2.0,yaw:1.5}}" --feedback

TO USE DRAW A SQUARE
terminal 2:
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtle_control --symlink-install
source install/setup.bash
ros2 action list
ros2 run turtle_control square_action_server

terminal 3:
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtle_control --symlink-install
source install/setup.bash
ros2 run turtle_control reset_server


terminal 4:
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtle_interface --symlink-install
source install/setup.bash
ros2 interface show turtle_interface/action/DrawSquare
ros2 action list -t
ros2 action send_goal /draw_square turtle_interface/action/DrawSquare "{side_length:2.0}" --feedback

TO USE THE RESET SERVICE
ros2 service call /reset_turtle std_srvs/srv/Trigger "{}"

TO READ CURRENT LINEAR SPEED 
ros2 param get /go_to_pose_server max_linear_speed

TO READ CURRENT ANGULAR SPEED
ros2 param get /go_to_pose_server max_angular_speed

TO CHANGE VALUES THROUGH THE CUSTOM SERVICE
ros2 service call /set_speed_limits turtle_interface/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.2}"

TO CHANGE PARAMETERS DIRECTLY
ros2 param set /go_to_pose _server max_linear_spedd=1.0
