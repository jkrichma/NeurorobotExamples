# Bee Navigation
# Jeff Krichmar, March 21 2021

This directory contains a Webots project to simulate how a honeybee navigates a narrow corridor. 
It is based on the balanced optic flow idea to stay in the middle of a corridor, which was proposed as a visual navigation strategy used by bees.  And maybe humans too.  

To make this work properly, the range of the horizontally oriented proximity sensors of the e-Puck robot were increased by a factor of 10. This was achieved by changing the proximity sensor lookup table in the Robot node.

For more details on the honey bee experiments, see:
    Srinivasan and Zhang (1997). Visual control of honeybee flight. ESS 84, 95-113.
    Srinivasan, Leherer, Kirchner, Zhang (1991). Range perception through apparent image speed in freely flying honeybees. Vis Neurosci 6, 519-535.   

