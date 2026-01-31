function dv = lagrangianDynamics(t, state, u)
    % Inputs:
    %           t: Simulation time
    %           state: State vector of every agent
    %           u: Control signal input
    % Output:
    %           dv: acceleration of every agent
    %
    % Description:
    %           This script is consisted with the dynamics of every agent.
    %           The dynamics are based on a fixed-wing VTOL UAV.

    global N
    global n
    global C
    global G
    global v_dot
    global Mk
    
    vel = state((n*N+1):2*n*N);
  
    v_dot = inv(Mk)*(u + disturbance(t) - C*vel - G);
    
    dv = v_dot;
end

