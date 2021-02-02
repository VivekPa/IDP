"""receiver controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, GPS, Emitter, Receiver
#from subdir.functions import getBearing, getDistanceandRotation, moveTo
import math
import struct #to convert native python data types into a string of bytes and vice versa
import numpy as np


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

leftMotor = robot.getDevice('wheel1')
rightMotor = robot.getDevice('wheel2')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

ds_left = robot.getDevice('ds_left')
ds_left.enable(TIME_STEP)
ds_right = robot.getDevice('ds_right')
ds_right.enable(TIME_STEP)

emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)
list_of_messages = []
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    print('Receiver Queue length:'  , receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message=receiver.getData()
        dataList=struct.unpack("3f",message)
        list_of_messages.append(list(dataList))
        print(dataList)
        receiver.nextPacket() #deletes the head packet
    # Process sensor data here.
    else:
        print('no message','length of message list: {}'.format(len(list_of_messages)))
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
 