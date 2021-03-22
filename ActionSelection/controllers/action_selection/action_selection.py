"""
action_selection.py

    Author     : Jeff Krichmar
    Date       : 16 February 2021
    Description :
    * Simulation of Subsumption Architecture using states and events.
    * Webot description:  Uses the Khepera robot with camera, distance sensors and gripper
    *                     to Explore, Eat and Hide in the environment.
    *                     There was a tradeoff between hunger and fear levels. On early trials, the
    *                     fear level was high. As the trials progressed, the hunger level increased.
    *                     After eating, the hunger level was reset.
    *                     The explore behavior was simulated with a random walk that switched between
    *                     bouts of moving forward and tumbling (random turns).

"""


#####################################################################
# Imports
#####################################################################
from controller import Robot, Camera, DistanceSensor, Motor, Supervisor
from random import random, seed
from datetime import datetime
import math


#####################################################################
# Constants
#####################################################################
TIME_STEP = 64

# Khepera constanst for distance sensors, motors, gripper, and camera
NUM_DISTANCE_SENSORS = 8
CLOSE_DISTANCE = 850
NEAR_DISTANCE = 500
OPEN_GRIP = 0.029
CLOSED_GRIP = 0.005
NUM_SENSORS = 8
FRONT_LEFT = 2
FRONT_RIGHT = 3
BASE_SPEED = 5.0
TURN_RATE = 2.0
SEE_OBJECT = 20
BETA = 0.10
TRIAL_LENGTH = 20
fear_level = TRIAL_LENGTH
hunger_level = 0


# Timers for gripping and hiding
grip_timer = 0
HIDE_TIMEOUT = 100
STICK_HEIGHT = 0.03
hide_timer = 0

FORWARD_MAX = 50
TUMBLE_MAX = 20

# States
STATE_EXPLORE = 0
STATE_EAT = 1
STATE_HIDE = 2
state = STATE_EXPLORE

# Events
EVENT_OBJECT = 0
EVENT_FULL = 1
EVENT_CURIOUS = 2
events = [False, False, False]

#####################################################################
# Functions
#####################################################################
    
def get_distance(x1, x2, y1, y2):
    """
       Calculate the Euclidean distance
       - Parameters
           x1, x2 - First and second element of the x vector
           y1, y2 - First and second element of the y vector
       - Returns the distance between x and y
    """
    return math.sqrt(pow(x1-y1,2.0)+pow(x2-y2,2.0))

def get_object():
    """
       Detect a red object from the camera image
       - Returns the object position and size
    """
    obj[0] = 0.0 # object position 
    obj[1] = 0.0 # object size 
    image = camera.getImage()
    for i in range(width):
        for j in range(height):
            red = camera.imageGetRed(image, width, i, j)/256.0
            blue = camera.imageGetBlue(image, width, i, j)/256.0
            green = camera.imageGetGreen(image, width, i, j)/256.0
            if (red > 2*green and red > 2*blue):
                obj[0] += (i - width/2.0)/width
                obj[1] += 1
    return obj
    
def action_select (a0, a1, beta):
    """
       Calculate the Softmax function to choose an action. Converts a0 and a1 into a probability distribution
       - Parameters
           a0, a1 - values for action 0 or action 1
           beta - temperature for Softmax function
       - Returns true if a0 selected and false if a1 is selected
    """
    p = math.exp(beta*a0)/(math.exp(beta*a0)+math.exp(beta*a1));

    if random() < p:
        act = True
    else:
        act = False
            
    return act
    
def explore(ds):
    """
       Explore state - Implement a random walk
       - Returns the wheel velocity
    """
    global forward_timer
    global tumble_timer
    global tumble_direction
    
    velo = [BASE_SPEED, BASE_SPEED]
    
    # Case 1: Near a wall (Head on collision). Spin clockwise
    if ds[FRONT_LEFT] > NEAR_DISTANCE and ds[FRONT_RIGHT] > NEAR_DISTANCE:
         velo = [TURN_RATE, -TURN_RATE]

    # Case 2: Wall is near on the left side of the robot. Spin clockwise
    elif ds[0] > NEAR_DISTANCE or ds[1] > NEAR_DISTANCE or ds[2] > NEAR_DISTANCE:
         velo = [TURN_RATE, -TURN_RATE]

    # Case 3: Wall is near on the right side of the robot. Spin counterclockwise   
    elif ds[3] > NEAR_DISTANCE or ds[4] > NEAR_DISTANCE or ds[5] > NEAR_DISTANCE:
         velo = [-TURN_RATE, TURN_RATE]

    # Case 4: Go forward
    elif forward_timer > 0:
        velo = [BASE_SPEED, BASE_SPEED]
        forward_timer -= 1    # decrement forward counter
        
        # after moving forward, set a timer for the tumble (spin)
        if forward_timer == 0:
            tumble_timer = 10+int(random()*TUMBLE_MAX)
            if random() > 0.5:
               tumble_direction = 1
            else:
               tumble_direction = -1
        
    # Case 5: Tumble (spin)
    elif tumble_timer > 0:
        velo = [TURN_RATE*tumble_direction, -TURN_RATE*tumble_direction]
        tumble_timer -= 1    # decrement tumble timer
        
        # after tumbling, move forward
        if tumble_timer == 0:
            forward_timer = 10+int(random()*FORWARD_MAX)
    
    return velo

def eat(ds, food):
    """
       Eat state - approach and pick up red stick
       - Parameters
           ds - distance sensors from Khepera robot
           food - position of the red stick
       - Returns the wheel velocity
    """
    global grip_timer
    global events 
    
    velo = [0.0,0.0]
    
    # if close to object, open the gripper
    if (grip_timer == 0) and ((ds[FRONT_LEFT] >  CLOSE_DISTANCE) or (ds[FRONT_RIGHT] >  CLOSE_DISTANCE)):
      left_grip.setPosition(OPEN_GRIP)
      right_grip.setPosition(OPEN_GRIP)
      grip_timer += 1;

    # sequence to pick up and put down gripper 
    elif grip_timer > 0:
        grip_timer += 1

        # gripping has started. move arm down
        if (grip_timer == 20):
            motor.setPosition(0.0) # arm down
      
        # close gripper on the object
        elif (grip_timer == 40):
            left_grip.setPosition(CLOSED_GRIP)
            right_grip.setPosition(CLOSED_GRIP)
      
        # lift object up
        elif (grip_timer == 80):
            motor.setPosition(-1.4)
        
        # put object down
        elif (grip_timer == 100):
            motor.setPosition(0.0)
        
        # open gripper to release object
        elif (grip_timer == 120):
            left_grip.setPosition(OPEN_GRIP)
            right_grip.setPosition(OPEN_GRIP)

        # raise arm
        elif grip_timer == 140:
            motor.setPosition(-1.4)

        # close gripper. place red stick in new position
        elif grip_timer == 160:
            left_grip.setPosition(CLOSED_GRIP)
            right_grip.setPosition(CLOSED_GRIP)
            
            # make sure red stick is at least 0.25 meters from robot
            robot_pos = translation_robot_field.getSFVec3f()
            x = robot_pos[0]
            z = robot_pos[2]
            while get_distance(robot_position[0],robot_position[2],x,z) < 0.25:
                x = 0.5*random()-0.25
                z = 0.5*random()-0.25
            translation_stick_field.setSFVec3f([x, STICK_HEIGHT, z])
            rotation_stick_field.setSFRotation(stick_rotation)
            
            # robot ate red stick. now full
            events[EVENT_FULL] = True
    
    # lost sight of object, spin around        
    elif food[1] < SEE_OBJECT:
        velo[0] = TURN_RATE
        velo[1] = -TURN_RATE
    
    # tracking the object based on its size and position. the smaller the object, the higher the turn rate    
    else:
        turn = food[0]/food[1]*TURN_RATE
        velo[0] = BASE_SPEED+turn
        velo[1] = BASE_SPEED-turn            
            
    return velo   
    
def hide(ds, obj):
    """
       Hide state - avoid red stick, find a wall and hide by the wall
       - Parameters
           ds - distance sensors from Khepera robot
           food - position of the red stick
       - Returns the wheel velocity
    """

    global hide_timer
    global events
    
    velo = [0.0, 0.0]

    # check all distance sensors to detect if near a wall    
    near_wall = False
    for i in range(NUM_SENSORS):
        if ds[i] > CLOSE_DISTANCE:
            near_wall = True
    
    # wall has not been found        
    if hide_timer == 0:
    
        # near a wall, start hiding timer
        if near_wall:
            velo[0] = 0.0
            velo[1] = 0.0
            hide_timer += 1
 
        # rotate until object is out of view
        elif obj[1] > SEE_OBJECT:
            velo[0] = -TURN_RATE
            velo[1] = TURN_RATE
        
        # drive straight to find wall       
        else:
            velo[0] = BASE_SPEED
            velo[1] = BASE_SPEED
    
    # wall has been found. stay still until timeou
    else:
        velo[0] = 0.0
        velo[1] = 0.0
        hide_timer += 1
        
        # after timeout, roobt is curious
        if hide_timer == HIDE_TIMEOUT:
            events[EVENT_CURIOUS] = True
                        
    return velo
    
def state_transition (e, s):
    """
       Transition to new state based on an event and current state
       - Parameters
           e - environmental event
           s - current state
       - Returns new state
    """

    global forward_timer
    global grip_timer
    global hide_timer
    global tumble_timer
    global fear_level
    global hunger_level
    
    # Check if robot is hungry based on the levels of fear and hunger. 
    # Use the softmax function to decide if hungry
    hungry = action_select(hunger_level, fear_level, BETA)
    
    # if currently exploring and there is an object event and the robot is hungry
    #     transition to the Eat state
    if s == STATE_EXPLORE and e[EVENT_OBJECT] and hungry:
        new_state = STATE_EAT
        grip_timer = 0
        
    # if currently exploring and there is an object event and the robot is scared
    #     transition to the Hide state
    elif s == STATE_EXPLORE and e[EVENT_OBJECT]:
        new_state = STATE_HIDE
        hide_timer = 0

    # if the robot has finished eating
    #     reset the hunger level
    #     transition to the Explore state
    elif s == STATE_EAT and e[EVENT_FULL]:
        new_state = STATE_EXPLORE
        forward_timer = int(random()*FORWARD_MAX)
        tumble_timer = 0
        hunger_level = 0
        
    # if the robot has finished hiding and is curious
    #     transition to the Explore state
    elif s == STATE_HIDE and e[EVENT_CURIOUS]:
        new_state = STATE_EXPLORE
        forward_timer = int(random()*FORWARD_MAX)
        tumble_timer = 0
   
    # stay in the current state   
    else:
        new_state = s
    
    return new_state

#####################################################################
# Main Routine
#####################################################################

# Use the Supervisor to create an instance for the robot and the red stick.
supervisor = Supervisor()
robot_node = supervisor.getFromDef("Khepera")
stick_node = supervisor.getFromDef("RED_STICK")

# setup motors and set target position to infinity (speed control).
left_motor = supervisor.getDevice('left wheel motor')
left_motor.setPosition(float('+inf'))
left_motor.setVelocity(0.0)
right_motor = supervisor.getDevice('right wheel motor')
right_motor.setPosition(float('+inf'))
right_motor.setVelocity(0.0)

# set up gripper
motor = supervisor.getDevice("motor");
left_grip = supervisor.getDevice("left grip");
right_grip = supervisor.getDevice("right grip");

# Get the camera device, enable it, and store its width and height
camera = supervisor.getDevice('camera')
camera.enable(TIME_STEP)
width = camera.getWidth()
height = camera.getHeight()

# set up and enable Khepera distance sensors
ds = []
dsNames = ['ds0', 'ds1', 'ds2', 'ds3', 'ds4', 'ds5', 'ds6', 'ds7']
for i in range(NUM_DISTANCE_SENSORS):
    ds.append(supervisor.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)


# Get the red stick's translation (position) and rotation (orientation) fields
# Save the intitial translation and rotation of the robot.  This will be 
# used to return the robot to the starting position after each trial.
translation_stick_field = stick_node.getField("translation")
rotation_stick_field = stick_node.getField("rotation")
stick_rotation = [0.0, -1.0, 0.0, 3.1416]

# Get the robot's translation (position) and rotation (orientation) fields
translation_robot_field = robot_node.getField("translation")
rotation_robot_field = robot_node.getField("rotation")

obj = [0.0, 0.0]
grip_timer = 0
speed = [0.0, 0.0]
state_transitions = 0
ts = 0
state_name = ""

# set the random number generator. by setting the seed to the current time
# it is extremely rare to get a repeated sequence.
seed(datetime.now())

# metrics file will have the time step, state and robot position
f = open("trial_metrics.txt", "w")
f.write("TimeStep\tTransitions\tState\tFearLevel\tHungerLevel\tPosX\tPosZ\n")

# Main loop:
# - perform simulation steps until the robot has transitioned through TRIAL_LENGTH states
while supervisor.step(TIME_STEP) != -1 and state_transitions < TRIAL_LENGTH:
 
    # read the distance sensors 
    dsValues=[]
    for i in range(NUM_DISTANCE_SENSORS):
        dsValues.append(ds[i].getValue())
    
    # use the camera to detect an object event    
    obj = get_object()    
    if obj[1] > SEE_OBJECT:
        events [EVENT_OBJECT] = True
    else:
        events [EVENT_OBJECT] = False
    
    # get a new state based on the current state and events    
    old_state = state
    state = state_transition (events, old_state)
    
    # clear the states for this time step    
    events = [False, False, False]
     
    # execute behavior based on current state   
    if state == STATE_EXPLORE:
        speed = explore(dsValues)
        state_name = "Explore"
        
    elif state == STATE_EAT:
        speed = eat(dsValues, obj)
        state_name = "Eat"

    elif state == STATE_HIDE:
        speed = hide(dsValues, obj)
        state_name = "Hide"
            
    left_motor.setVelocity(speed[0])
    right_motor.setVelocity(speed[1])
    
    # write out the time step, state, and robot position
    robot_position = translation_robot_field.getSFVec3f()
    f.write("%i\t%i\t%s\t%i\t%i\t%4.4f\t%4.4f\n" % (ts, state_transitions, state_name, fear_level, hunger_level, robot_position[0], robot_position[2]))

    ts += 1
    
    if old_state != state:
        state_transitions += 1
        hunger_level += 1 # increase hunger
        fear_level -= 1   # decrease fear

        print("%i %s\n" % (state_transitions, state_name))
        

f.close()
