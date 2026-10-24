# design_of_mechatronic_systems_lab
Name: Ahmed Khaled  ID:9275
 Packages structure:
 ROS 2 Workspace
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
