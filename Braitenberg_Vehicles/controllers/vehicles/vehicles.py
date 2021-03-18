"""vehicles controller."""


#
#  Braitenberg Vehicles
#  NAME: Jeff Krichmar
#  DATE: January 15, 2021

 #
 # Braitenberg Vehicles Demonstration    
 # Description:  An example of Braitenberg vehicles 2 and 3 using a light sensor device.
 #               Inhibition achieved by subtracting the light sensor reading from the maximum possible light value
# 

# import  classes of the controller module for the motors and light sensors
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, LightSensor, Motor

# create the Robot instance.
robot = Robot()

print('You can move the light using your mouse, the robot will react to it')


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialize motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# initialize light sensors
leftLightSensor = robot.getDevice('ls0') # left light sensor
leftLightSensor.enable(timestep)
rightLightSensor = robot.getDevice('ls1') # right light sensor  
rightLightSensor.enable(timestep)

# Some constants
MAX_SPEED = 10.0    # maximum speed for this robot
MAX_LIGHT = 1024.0  # maximum value for the light sensors
MOTOR_SCALE_FACTOR = 100.0

vehicleName = "2A"  # vehicle name can be a string equal to 2A, 2B, 3A, or 3B

leftSpeed = 1.0    # initial speed
rightSpeed = 1.0   # always good practice to initialize variables


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the light sensors:
    leftLight = leftLightSensor.getValue()
    rightLight = rightLightSensor.getValue()
 
    # Vehicle 2A - Fear
    #    Ipsilateral excitatory connections
    #    Keep the motor values between 0 and 10 by dividing the numerator by a scale factor (100). 
    if vehicleName == "2A":
        leftSpeed = leftLight / MOTOR_SCALE_FACTOR   # left light sensor connected to left motor
        rightSpeed = rightLight / MOTOR_SCALE_FACTOR # right light sensor connected to right motor
    
    # Vehicle 2B - Aggression
    #    Contralateral excitatory connections
    #    Keep the motor values between 0 and 10 by dividing the numerator by a scale factor (100). 
    elif vehicleName == "2B":
        leftSpeed = rightLight / MOTOR_SCALE_FACTOR  # right light sensor connected to left motor
        rightSpeed = leftLight / MOTOR_SCALE_FACTOR  # left light sensor connected to right motor
    
    # Vehicle 3A - Lover
    #    Ipsilateral inhibitory connections
    #    Subtract the light sensor value from the maximum possible light sensor reading. 
    #    Now the value in the numerator is 0 for bright light and MAX_LIGHT (1024) when there is no light.
    #    Keep the motor values between 0 and 10 by dividing the numerator by a scale factor (100). 
    elif vehicleName == "3A":
        leftSpeed = (MAX_LIGHT - leftLight) / MOTOR_SCALE_FACTOR   # left light sensor connected to left motor
        rightSpeed = (MAX_LIGHT - rightLight) / MOTOR_SCALE_FACTOR # right light sensor connected to right motor
    
    # Vehicle 3B - Explorer
    #    Contralateral inhibitory connections
    #    Subtract the light sensor value from the maximum possible light sensor reading. 
    #    Now the value in the numerator is 0 for bright light and MAX_LIGHT (1024) when there is no light.
    #    Keep the motor values between 0 and 10 by dividing the numerator by a scale factor (100). 
    elif vehicleName == "3B":
        leftSpeed = (MAX_LIGHT - rightLight) / MOTOR_SCALE_FACTOR # right light sensor connected to left motor
        rightSpeed = (MAX_LIGHT - leftLight) / MOTOR_SCALE_FACTOR # left light sensor connected to right motor
    
    else:
        print("Bad vehicle name " + vehicleName)   
    
    # Check for boundary conditions.  Do not let speed be greater than
    # the maximum speed for the robot
    if leftSpeed > MAX_SPEED:
        leftSpeed = MAX_SPEED
        
    if rightSpeed > MAX_SPEED:
        rightSpeed = MAX_SPEED 

    # Set the velocity of the wheels based on the calculations above
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # Print out the values of the light sensors and motor speeds
    #    Since these are floating point values, they must be converted to strings with the str command
    #    Use the round command to only print two digits after the decimal point
    print("leftLight = " + str(round(leftLight,2)) + " leftSpeed = " + str(round(leftSpeed,2)))  
    print("rightLight = " + str(round(rightLight,2)) + " rightSpeed = " + str(round(rightSpeed,2))) 



