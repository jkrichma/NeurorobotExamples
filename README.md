# Neurorobot Examples
# Jeff Krichmar, March 19, 2021
#    September 9 2022: Updated for Webots R2022a.  There may be warnings on startup.  But should still work fine.
#    December 23, 2022: Updated for Webots R2023a.  Got rid of most warnings.  Note that with Morris Water Maze you may need to re-select the C controller and change the seed. 
These examples illustrate concepts discussed in the book "Neurorobotics: Connecting the Brain, Body and Environment." Tiffany Hwu and Jeff Krichmar, Cambridge, MA, MIT Press. The examples use the Webots robot simulator (https://cyberbotics.com/). 

1) Braitenberg vehicles 2 and 3.
2) Bee navigation by balancing the flow across the left and right distance sensors.
3) Conditioning using Rescorla Wagner learning to tie value to different colors.
4) Double T maze navigation using Q learning.
5) Event-driven action selection.
6) Solving the Morris Water Maze using temporal difference learning.

All controllers were written in Python except for the Morris water maze, which was written in C.
 
