# IDP
Integrated Design Project 

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
```
controllers/
- blue_controller/ (same as pre_planned_path)
- pre_planned_path/
--- pre_planned_path.py
--- paths/
---- horizontal_traverse/
--- utils/
---- astarplanner.py
---- motion_api.py
---- obstacle_detection.py
---- path_planning.py
---- sensors_api.py
---- variables.py
protos/
- BlueHippo.proto
- RedHippo.proto
tools/
worlds/
- world.wbt
```

## References
Parts of this code are based on PythonRobotics https://github.com/AtsushiSakai/PythonRobotics
