"""bee_navigation controller."""

#
# File:          bee_navigation.py
# Date:  October 20, 2020
# Description: Use the balanced optic flow idea to stay in the middle of a corridor. 
#   Srinivasan and Zhang (1997). Visual control of honeybee flight. ESS 84, 95-113.
#   Srinivasan, Leherer, Kirchner, Zhang (1991). Range perception through apparent image speed in freely flying honeybees. Vis Neurosci 6, 519-535.   
# 
#  Increased the distance sensitivity of the horizontally oriented proximity sensors (1, 2, 5 and 6) by a factor of 10
#
# Author: J.L. Krichmar

from controller import Robot, DistanceSensor, Motor

import math

MAX_SPEED = 6.28
VELO_GAIN = 0.25
BALANCE = True

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
# enable sensors
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# left and right wheels
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    psValues = [0] * len(ps)
    for i in range(8):
        psValues[i] = ps[i].getValue()
        
    # initialize motor speeds at 50% of MAX_SPEED
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    # calculate a population code based on the readings of 6 distance sensors
    ofLeft = 0
    ofRight = 0
    
    for i in range(3):
        ofRight += psValues[i]
        
    for i in range(5,8):
        ofLeft += psValues[i]
        
    # calculate the turning rate based on the balancing algorithm from the proximity sensors.
    # the idea is to balance the readings from the left and right side   
    turnRate = VELO_GAIN*(ofLeft-ofRight)/(ofLeft+ofRight)
          
    # Add the population code velocity to the forward speed
    leftSpeed += turnRate*leftSpeed;
    rightSpeed += -turnRate*rightSpeed;

        
    # Check for boundary conditions.  Do not let speed be greater than
    # the maximum speed for the robot
    if leftSpeed > MAX_SPEED:
        leftSpeed = MAX_SPEED
        
    if rightSpeed > MAX_SPEED:
        rightSpeed = MAX_SPEED 

    # Set the velocity of the wheels based on the calculations above
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
 
    print("%3.2f %3.2f %3.2f %3.2f %3.2f\n" % (turnRate, ofLeft, ofRight, leftSpeed, rightSpeed))

# Enter here exit cleanup code.
