# GNC-Swarm
Drone swarm controlled by one operator implemented and simulated in Simulink. Developed during course Guidance Navigation and Control, (F24) at SDU Odense.

The Simulink implementation consists of the following files:
```
Quadcopter\quadcopter_swarm_final_solution.slx
Quadcopter\quadcopter_swarm_final_solution_par.m
```

## Simulink implementation
The drone swarm is implemted in Simulink, controlled through a consensus algorithm following graph theory, making use of a fixed vertex for swarm positioning and displacement for interswarm positioning of each drone. Collision avoidance was implemented as well, with a solution to solve deadlocks cause by head-on collisions given the collision avoidance.

The drone swarm consists of 5 quadcopters using rigid-body dynamics and quaternion-based kinematics.
Each drone is controlled with feedback control using their own inner-loop controllers for altitude and axis rotation, as well as outer-loop controller for velocity along the x and y axis. The frame notation is in NED for the earth-fixed frame and FRD for the body-fixed frame.