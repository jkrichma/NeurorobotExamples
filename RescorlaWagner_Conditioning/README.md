# Rescorla-Wagner Conditioning
# Jeff Krichmar, March 19 2021
#    September 9 2022: Updated for Webots R2022a.  There may be a warning on startup.  But should still work fine.

This directory contains a Webots project to observe the Rescorla-Wagner learning rule in action.  The robot uses its camera to view different colored object. The red, green, and blue objects are rewarded at different rates. The more reward received, the longer the robot stares at the color. The simulation runs for 100 trials, where each trial is a pause and stare action by the robot.  After 50 trials, the reward contingencies change.  The simulation writes out the results in three files; red.txt, green.txt and blue.txt.
 
