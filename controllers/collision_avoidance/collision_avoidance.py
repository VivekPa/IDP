"""collision_avoidance controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, DistanceSensor, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

#initialize distance sensors
ds_left = robot.getDevice('ds_left')
ds_left.enable(TIME_STEP)
ds_right = robot.getDevice('ds_right')
ds_right.enable(TIME_STEP)

#initialize motors
leftMotor = robot.getDevice('wheel1')
rightMotor = robot.getDevice('wheel2')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    ds_leftvalue = ds_left.getValue()
    ds_rightvalue = ds_right.getValue()

    # Process sensor data here.
    # detect obstacles
    right_obstacle = ds_rightvalue < 800.0
    left_obstacle = ds_leftvalue < 800.0
    print(left_obstacle, right_obstacle)

    # Enter here functions to send actuator commands, like:
    MAX_SPEED = 6.28
    
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 1 * MAX_SPEED
        rightSpeed = -1 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -1 * MAX_SPEED
        rightSpeed = 1 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Enter here exit cleanup code.
