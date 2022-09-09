"""
mfrl_maze.py

    Author     : Jeff Krichmar
    Date       : 06 February 2021
    Title      : Model Free Reinforcement Learning in a Double-T Maze
    Description :
    * Simulation of Q learning in a maze.
    * Webot description:  Uses the e-Puck robot and its proximity sensors to detect maze walls.
    * Changes
    *** September 7, 2022. Changed the rotation values due to new coordinate system in Webots R2022a

"""


#####################################################################
# Imports
#####################################################################
from controller import Robot, DistanceSensor, Motor, Supervisor
from random import random, seed
import math


#####################################################################
# Functions
#####################################################################

def getDistance(x1, x2, y1, y2):
    """
       Calculate the Euclidean distance 
       - Parameters
           x1, x2 - First and second element of the x vector 
           y1, y2 - First and second element of the y vector
       - Returns the distance between x and y
    """
    return math.sqrt(pow(x1-y1,2.0)+pow(x2-y2,2.0))
    
def action_select (a0, a1, beta, returnAct):
    """
       Calculate the Softmax function to choose an action. Converts a0 and a1 into a probability distribution
       - Parameters
           a0, a1 - values for action 0 or action 1
           beta - temperature for Softmax function
           returnAct - if true, returns select action, otherwise returns the probability of taking action a0 
       - Returns the distance between x and y
    """
    p = math.exp(beta*a0)/(math.exp(beta*a0)+math.exp(beta*a1));

    if random() < p:
        act = 0
    else:
        act = 1
    
    if returnAct:   
        return act
    else:
        return p

def print_table (a, b, s, q_tbl, probability):
    """
       Print either the state action table (q_tbl) or the probability of taking an action
       - Parameters
           a - alpha learning rate
           b - beta temperature
           s - random number seed
           q_tbl - state/action table
           probability - if true, print the probability of taking an action as calulated by Softmax
                         otherwise print the state/action table 
    """
    f = open("tbl_a" + str(a) + "_b" + str(b) + "_s" + str(s) + ".txt", "w")
    
    if probability:
        print("Probability of Turning")
        print("State\tLeft\tRight")
        for i in range(STATES):
            p = action_select (q_tbl[i][0],q_tbl[i][1],BETA, False);       
            print("%i\t%3.2f\t%3.2f\n" % (i, p, 1-p))
            f.write("%i\t%3.2f\t%3.2f\n" % (i, p, 1-p))
    else:
        print("Expected Value")
        print("State\tLeft\tRight")
        for i in range(STATES):
            print("%i\t%3.2f\t%3.2f\n" % (i, q_tbl[i][0], q_tbl[i][1]))
            f.write("%i\t%3.2f\t%3.2f\n" % (i, q_tbl[i][0], q_tbl[i][1]))
    
    f.close()

#####################################################################
# Constants
#####################################################################
TIME_STEP = 64
MAX_SPEED = 6.28
Reward3 = [-0.338, 0, 0.417]
Reward4 = [-0.338, 0, 0.008]
Reward5 = [0.265, 0, 0.008]
Reward6 = [0.265, 0, 0.417]
REWARD_DIST_THR = 0.05
STATES = 7
ACTIONS = 2
LEFT = 0
RIGHT = 1
TRIALS = 50
GAMMA = 0.9
TRIALS = 50
VELO_GAIN = 0.25
ALPHA = 0.5
BETA = 0.25
SEED = 1

#####################################################################
# Global Variables
#####################################################################
turnLeft = False
turnRight = False
turnTimer = 0

# Q is the state/action table
Q = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]

# T is the transition from one state to another depending on the action
T = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
T[0][LEFT] = 1
T[0][RIGHT] = 2
T[1][LEFT] = 3
T[1][RIGHT] = 4
T[2][LEFT] = 5
T[2][RIGHT] = 6

# R is the rewards for each state
R = [0,0,0,0,3,5,0]

current_state = 0
turnLeft = 0
turnRight = 0
trialComplete = False
trial = 0


# Use the Supervisor to create the Robot instance.
supervisor = Supervisor()
robot_node = supervisor.getFromDef("epuck")

# Get the robot's translation (position) and rotation (orientation) fields
# Save the intitial translation and rotation of the robot.  This will be 
# used to return the robot to the starting position after each trial.
translation_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")
initial_translation = translation_field.getSFVec3f()
initial_rotation = [-0.577, 0.577, 0.577, 2.09]


# initialize proximity sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(supervisor.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

# initialize motors in supervisor mode
leftMotor = supervisor.getDevice('left wheel motor')
rightMotor = supervisor.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

seed(SEED)

# Run the simulation for a number of trials
while trial < TRIALS: 
    supervisor.step(TIME_STEP)
    
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # When the robot reaches a T junction, the two front proximity sensory
    # will have high values 
    front_collision = psValues[0] > 80.0 and psValues[7] > 80.0
    
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED

    # Use the balance equation to calculate the turn rate and set the wheel speed
    ofLeft = 0
    ofRight = 0   
    for i in range(3):
        ofRight += psValues[i]       
    for i in range(5,8):
        ofLeft += psValues[i]
    turnRate = VELO_GAIN*(ofLeft-ofRight)/(ofLeft+ofRight)
    leftSpeed += turnRate*leftSpeed;
    rightSpeed += -turnRate*rightSpeed;

    # Navigate the maze and update Q learning rule. Special cases.    
    #     Case 1: Robot is turning left for several time steps
    if turnLeft > 0:
        leftSpeed = -1.0
        rightSpeed = 1.0
        turnLeft = turnLeft - 1

    #     Case 2: Robot is turning right for several time steps
    elif turnRight > 0:
        leftSpeed = 1.0
        rightSpeed = -1.0
        turnRight = turnRight - 1

    #     Case 3: Robot is at the T-junction.
    #             a) Select a left or right action
    #             b) Get the current state based on that action
    #             c) Update the state/action table using Q learning
    elif front_collision:
        act = action_select(Q[current_state][0],Q[current_state][1],BETA,True);
        previous_state = current_state
        current_state = T[previous_state][act]
        if Q[current_state][LEFT] > Q[current_state][RIGHT]:
            maxQ = Q[current_state][LEFT]
        else:
            maxQ = Q[current_state][RIGHT]       
        Q[previous_state][act] = Q[previous_state][act] + ALPHA*(R[current_state] + GAMMA*maxQ - Q[previous_state][act])
        
        # Set a timer for the turn
        if act == 0:
            turnLeft = 20
        else:
            turnRight = 20
   
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # Get the current position of the robot. Check if the robot is near one of the end arms.
    # If the robot is near an end arm, this trial is complete
    current_translation = translation_field.getSFVec3f()
    distToR3 = getDistance(current_translation[0],current_translation[2],Reward3[0],Reward3[2])
    distToR4 = getDistance(current_translation[0],current_translation[2],Reward4[0],Reward4[2])
    distToR5 = getDistance(current_translation[0],current_translation[2],Reward5[0],Reward5[2])
    distToR6 = getDistance(current_translation[0],current_translation[2],Reward6[0],Reward6[2])
    
    if distToR3 < REWARD_DIST_THR:
        trialComplete = True
        print('Trial ' + str(trial) + ': Reached R3 - ' + str(R[3]) + ' points')

    elif distToR4 < REWARD_DIST_THR:
        trialComplete = True
        print('Trial ' + str(trial) + ': Reached R4 - ' + str(R[4]) + ' points')

    elif distToR5 < REWARD_DIST_THR:
        trialComplete = True
        print('Trial ' + str(trial) + ': Reached R5 - ' + str(R[5]) + ' points')

    elif distToR6 < REWARD_DIST_THR:
        trialComplete = True
        print('Trial ' + str(trial) + ': Reached R6 - ' + str(R[6]) + ' points')
      
    # If the trial is complete, clear and update trial parameters, reset the robots position and orientation 
    # to the maze start location  
    if trialComplete:
        trial += 1
        current_state = 0
        turnLeft = 0
        turnRight = 0
        trialComplete = False
        translation_field.setSFVec3f(initial_translation)
        rotation_field.setSFRotation(initial_rotation)
        robot_node.resetPhysics()

# Simulation is complete. Print out table
print('---------------')
print('Trials Complete: alpha=' + str(ALPHA) + ' beta=' + str(BETA))
print_table(ALPHA, BETA, SEED, Q, True)    
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
    
