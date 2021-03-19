"""
rescorlawagner_conditioning.py

    Author     : Jeff Krichmar
    Date       : 22 January 2021
   Description     :
    * Demonstration of Rescorla Wagner learning rule.
    *    - Robot gets a reward when it looks at a red object (90% first 50 trials, 10% second 50 trials)
    *    - Robot gets a reward when it looks at a green object (25% first 50 trials, 100% second 50 trials)
    *    - Robot gets a reward when it looks at a blue object (50% first 50 trials, 50% second 50 trials)
    * Webot description:  This is an example of use of a camera device.

"""

#####################################################################
# Imports
#####################################################################
from __future__ import division
from random import random

from controller import Robot, Motor, Camera



#####################################################################
# Constant Variables
#####################################################################

SPEED = 4
LEARNING_RATE = 0.1
MIN_STARING_TIME = 10
NUMBER_OF_TRIALS = 100
COLOR_THRESHOLD = 3




#####################################################################
# Variables
#####################################################################
pause_counter = 0

color_names = ['red', 'green', 'blue']
ansi_colors = []

delta = 2.0

redRewardRateFirstHalf = 0.9
greenRewardRateFirstHalf = 0.25 # gets reward 90% of the time
blueRewardRateFirstHalf = 0.5

redRewardRateSecondHalf = 0.1
greenRewardRateSecondHalf = 1.0 # gets reward 10% of the time
blueRewardRateSecondHalf = 0.5

wRed = 0.0
wGreen = 0.0 
wBlue = 0.0

#####################################################################
# Robot Initialization
#####################################################################
# Create the Robot instance.
robot = Robot()  # wb_robot_init();

# get the time step of the current world.
time_step = int(robot.getBasicTimeStep())

defaultStareTime = 1280/time_step

# setup motors and set target position to infinity (speed control).
left_motor = robot.getDevice('left wheel motor')
left_motor.setPosition(float('+inf'))
left_motor.setVelocity(0.0)
right_motor = robot.getDevice('right wheel motor')
right_motor.setPosition(float('+inf'))
right_motor.setVelocity(0.0)

# Get the camera device, enable it, and store its width and height
camera = robot.getDevice('camera')
camera.enable(time_step)
width = camera.getWidth()
height = camera.getHeight()

# Only use the center third of the image. This will cut out the sky and floor
widthStart = int(width / 3)
widthEnd = int(2 * width / 3)
heightStart = int(height / 2)
heightEnd = int(3 * height / 4)

#####################################################################
# File Output (color data)
#####################################################################
# This is how you open a file to write information to...
fRed = open('red.txt', 'w')
fBlue = open('blue.txt', 'w')
fGreen = open('green.txt', 'w')

# Initial trial counters
trials = 0 
trialsRed = 0
trialsBlue = 0 
trialsGreen = 0

#####################################################################
# Main loop:
# - perform simulation steps until Webots is stopping the controller
#####################################################################
while trials < NUMBER_OF_TRIALS:
# while robot.step(time_step) != -1:

    robot.step(time_step)
    
    if (trials < NUMBER_OF_TRIALS/2):
        redRwd = redRewardRateFirstHalf
        greenRwd = greenRewardRateFirstHalf
        blueRwd = blueRewardRateFirstHalf
    else:
        redRwd = redRewardRateSecondHalf
        greenRwd = greenRewardRateSecondHalf
        blueRwd = blueRewardRateSecondHalf

    # Get the new camera values
    image = camera.getImage()

    # Decrement the pause_counter
    if (pause_counter > 0):
        pause_counter = pause_counter - 1

    #
    #  Case 1
    #      A color object was found recently. The robot waits in front of it until pause_counter
    #      is decremented enough
    #  
    if (pause_counter > 640 / time_step):
        left_speed = 0
        right_speed = 0
    # 
    #  Case 2
    #      Look for a new colored object. The robot begins to turn but don't analyse the image for a while,
    #      otherwise the same blob would be found again
    elif (pause_counter > 0):
        left_speed = -SPEED
        right_speed = SPEED
    # 
    #  Case 3
    #      The robot turns and analyses the camera image in order to find a new colored object
    else:   
        # Reset the sums
        red = 0
        green = 0
        blue = 0
 
        #
        #  Analyze the image from the camera. The goal is to detect a colored object in the camera view. 
        #  Count the values of the red, green and blue pixels
        for i in range(widthStart, widthEnd):
            for j in range(heightStart, heightEnd):
                red += camera.imageGetRed(image, width, i, j)
                blue += camera.imageGetBlue(image, width, i, j)
                green += camera.imageGetGreen(image, width, i, j)
        # 
        #  If one color is mch more represented than the others, the object is detected
        #      If a color blob is detected, apply the Rescorla Wagner learning rule to update the weight.
        #      The weight is updated with the delta rule based on if the reward is delivered,
        #      and how much the weight differs from the reward. If the reward is not delivered,
        #      the weight will decrease.  The time the robot stares at the color is proportional to the weight.
        # 
        
        pRwd = random() # probability of receiving a reward. random returns a number between 0 and 1.
 
        if ((red > COLOR_THRESHOLD*green) and (red > COLOR_THRESHOLD*blue)):
            current_color = 'red'
            wRed += LEARNING_RATE*((pRwd < redRwd)*delta - wRed) # update the weight according to the Rescorla Wagner learning rule
            pause_counter = int(defaultStareTime * wRed)
            
            # Write the trial and how long it looked at the object to the file.
            #     The string "\n" creates a new line.
            fRed.write(str(trials) + " " + str(pause_counter) + "\n")
            
        elif ((green > COLOR_THRESHOLD*red) and (green > COLOR_THRESHOLD*blue)):
            current_color = 'green'
            wGreen += LEARNING_RATE*((pRwd < greenRwd)*delta - wGreen)
            pause_counter += int(defaultStareTime * wGreen)
            fGreen.write(str(trials) + " " + str(pause_counter) + "\n")
            
        elif ((blue > COLOR_THRESHOLD*red) and (blue > COLOR_THRESHOLD*green)):
            current_color = 'blue'
            wBlue += LEARNING_RATE*((pRwd < blueRwd)*delta - wBlue)
            pause_counter = int(defaultStareTime * wBlue)
            fBlue.write(str(trials) + " " + str(pause_counter) + "\n")
        else:
            current_color = None

        if (pause_counter < MIN_STARING_TIME):
            pause_counter = MIN_STARING_TIME

        if (current_color is None):
            left_speed = -SPEED
            right_speed = SPEED

        else:
            left_speed = 0.0
            right_speed = 0.0
            trials += 1
            print("Trial " + str(trials) + " staring at " + str(current_color) + " for " + str(pause_counter) + " timesteps.") 

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


# Close the files.  If you don't close them, data might be lost.
fRed.close()
fBlue.close()
fGreen.close()

print("Completed all " + str(NUMBER_OF_TRIALS) + " trials!!  Still need to pause simulation.")
