# Optimal Distributed Control of Heterogeneous Multi-Agent Systems with Non-Convex Cost Functions and Sampled Adjacent Information

Master of Science in  Automation Systems - Field of Automatic Control Systems and Robotics.

National Technical University of Athens.

School of Electrical and Computer Engineering.

Division of Signals, Control and Robotics.

Control and Desicion Theory Laboratory.

## Abstract

Multi-agent systems inherently face synchronization and communication constraints, which
make their control particularly challenging. Within this framework, the optimal consensus
problem is a key topic of interest. In this problem, each agent is assigned a cost function
dependent on its output vector, and information is exchanged among neighboring agents
according to the communication graph topology. The objective is to drive all agents’ outputs
to a common value that minimizes the sum of these cost functions.
  This thesis investigates the optimal consensus problem for systems with non-convex cost
functions and connected communication topologies. Considering bandwidth limitations,
agents exchange position vectors at discrete sampling intervals. A regulation algorithm
based on NOCGPROX variables, representing the agents’ position errors, is proposed. Taking
into account the individual dynamics of each agent, distributed robust control laws are
designed to ensure convergence of the outputs to the optimal point. These laws combine the
sliding surface method, to achieve robustness, with feedback linearization, to compensate for
system nonlinearities.
  Theoretical analysis shows that the proposed control laws drive the error variables and
their squared integrals to zero, ensuring convergence to an optimal consensus point. The
algorithm and control laws were validated through simulations on a swarm of UAVs, demonstrating
accurate and reliable performance in MATLAB.

### Keywords:

Optimal Consensus, Distributed Non-Convex Optimization, Multi-Agent Systems, Asynchronous Communication, Distributed Automatic Control.
## Author
[George Krommydas](https://github.com/GeoKrom)
