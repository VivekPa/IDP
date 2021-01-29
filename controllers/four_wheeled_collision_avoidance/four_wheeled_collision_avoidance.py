"""four_wheeled_collision_avoidance controller."""

"""collision_avoidance controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, DistanceSensor, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# initialize motors
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheelsNames:
    wheels.append(robot.getDevice(name))



#initialize distance sensors
ds_left = robot.getDevice('ds_left')
ds_left.enable(TIME_STEP)
ds_right = robot.getDevice('ds_right')
ds_right.enable(TIME_STEP)

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
    
    # Enter here functions to send actuator commands, like:
    MAX_SPEED = 6.28
    
    for i in range(4):
        wheels[i].setPosition(float('inf'))
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # rotate right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.7 * MAX_SPEED
    elif right_obstacle:
        # rotate left
        leftSpeed  = -0.7 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        
    elif left_obstacle and right_obstacle:
        leftSpeed  = -0.7 * MAX_SPEED
        rightSpeed = -0.7 * MAX_SPEED
        wheels[0].setVelocity(leftSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[3].setVelocity(rightSpeed)
        leftSpeed  = 0.7 * MAX_SPEED
        rightSpeed = -0.7 * MAX_SPEED
        
    # write actuators inputs
    wheels[0].setVelocity(leftSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[3].setVelocity(rightSpeed)

# Enter here exit cleanup code.
