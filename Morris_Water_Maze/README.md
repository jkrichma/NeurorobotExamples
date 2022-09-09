# Morris Water Maze
# Jeff Krichmar, March 22 2021
#    September 9 2022: Updated for Webots R2022a. Compass is not working correctly.  Replaced compass with robot rotation from Supervisor.

Demonstration of Morris Water Maze (MWM). Builds on the demo code for obstacle avoidance on Firebird 6 robot by Anant Malewar; Nex Robotics.
 
Webots implementation of Foster, Dayan, Morris, "A Model of Hippocampally Dependent Navigation, Using the Temporal Difference Learning Rule", Hippocampus, 10:1-16, 2000.

An actor-critic model using the temporal difference rule learns the location of a platform. It uses the robot's compass to calculate its heading.  Proximity sensors are used to avoid the walls of the arena.  The actor-critic model learns a mapping between a place cell and a heading pointing towards the platform.

The simulation writes out 3 files in the controller directory:
- mwm_latency.txt contains the time to find the platform on each trial
- mwm_place.txt contains the centers of the place cells
- mwm_z.txt contains the actor weights after learning

These txt files can be loaded into MATLAB or some other program for analysis.  The file mwm_analysis.m contains a MATLAB script to plot the results.
