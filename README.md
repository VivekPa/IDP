# IDP
Integrated Design Project
Group L109
Engineers Across Borders

## Sensors
| Description  |  Variable  |
| ------------- | ------------- |
| GPS  | `gps`  |
| Compass  | `compass`  |
| Left Distance Sensor  | `ds_left`  |
| Right Distance Sensor  | `ds_right`  |
| Colour sensor  | `camera`  |
| Emitter  | `emitter`  |
| Reciever  | `receiver`  |

The compass is mounted such that the y-axis points up in the ENU coordinate system. (i.e. north points in the negative x axis).
GPS coordinates have their origin in the centre of the arena, which is on the x-z plane.

## Folder Structure

    ├── README.md                               <- The top-level README for using this project.
    │
    ├── protos                                  <- Webot models
    │   ├── RedHippo.proto
    │   └── BlueHippo.proto
    ├── tools                                   <- Empty
    │
    ├── worlds                                  <- World files
    │   └── world.wbt
    └── controllers                             <- controller code for this project
        ├── blue_controller                     <- controller for the blue robot. Has the same structure as pre_planned_path
        │
        ├── pre_planned_path                    <- controller for the red robot
            ├── pre_planned_path.py             <- main controller file
            ├── paths
            |   └── horizontal_traverse.txt     <- text file containing the pre-determined path
            └── utils
                └── __init__.py                 <- Makes utils a Python module
                └── astarplanner.py             <- Astar path planning code from PythonRobotics
                └── motion_api.py               <- Functions related to movement
                └── obstacle_detection.py       <- Functions related to detection of obstacles
                └── path_planning.py            <- Functions required to plan paths, uses astarplanner
                └── sensors_api.py              <- Functions related to usage of distance sensors, camers, compass and gps
                └── variables.py                <- Initializes the required variables

       
## References
Parts of this code are based on PythonRobotics https://github.com/AtsushiSakai/PythonRobotics
