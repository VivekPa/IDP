# IDP
Integrated Design Project 

## Sensors
| Description  |  Variable  |
| ------------- | ------------- |
| GPS  | `gps`  |
| Compass  | `compass`  |
| Left Distance Sensor  | `ds_1`  |
| Right Distance Sensor  | `ds_2`  |
| Colour sensor  | `camera`  |
| Emitter  | `emitter`  |
| Reciever  | `receiver`  |

The compass is mounted such that the y-axis points up in the ENU coordinate system. (i.e. north points in the negative x axis).
GPS coordinates have their origin in the centre of the arena, which is on the x-z plane. `get_gps_xz` is used to obtain only the x,z coordinate values. 

## Functions
| Function  |  Descriptions  |
| ------------- | ------------- |
| `getDistanceandRotation`  | <ul><li>Calculates the distance between current position and next position.</li><li>Calculates the angle between current heading and desired heading (i.e. required rotation angle)</li></ul>  |
| `moveTo` | Uses the output from `getDistanceandRotation` to calculate the desired bearing angle and and generates an error term by comparing the desired bearing angle with the bearing measurements from `compass`. A threshold value determines whether the vehicle should move straight or rotate. |
