# Optimal Distributed Control of Heterogeneous Multi-Agent Systems with Non-Convex Cost Functions and Sampled Adjacent Information

Master of Science in  Automation Systems - Field of Automatic Control Systems and Robotics.

National Technical University of Athens.

School of Electrical and Computer Engineering.

Division of Signals, Control and Robotics.

Control Systems Laboratory.

## Abstract

Multi-agent systems by their nature have synchronization and communication limitations that consequently render them challenging to control. A reliable solution is the problem of optimal consensus. Each agent is assigned with a cost function of the output vector, which is transmitted to neighboring agents through the topology of the communication graph. The goal is to drive the outputs of the agents to a common value that minimizes the sum of the cost functions. In this thesis, are assumed non-convex cost functions with connected topologies for the communication graph. The transmission of the position vectors between agents is done at specific sampling periods, in order to secure the information from system disturbances. A regulating algorithm was designed for this task through the NOCGPROX variables, which are the agent's position errors. Taking into consideration the dynamics of every agent, distributed robust control laws were designed to ensure that the outputs approach the optimal point or a region of it. These laws were developed with the sliding surface method to ensure robustness and feedback linearization to cancel the nonlinear dynamics of the system. These laws ensure that these variables tend to zero after a number of iterations, along with its square integration, for the precise implementation of this algorithm. The above algorithm and laws were applied to a swarm of UAVs. A number of simulation scenarios were conducted to ensure the accurate implementation and execution of the algorithm and control laws in MATLAB.

Keywords:

Optimal Consensus, Distributed Non-Convex Optimization, Multi-Agent Systems, Asynchronous Communication, Distributed Automatic Control.
## Author
[George Krommydas](https://github.com/GeoKrom)
