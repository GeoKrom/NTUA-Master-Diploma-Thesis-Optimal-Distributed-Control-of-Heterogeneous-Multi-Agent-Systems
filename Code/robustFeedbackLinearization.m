function u = robustFeedbackLinearization(t, pos, vel, sm, s_new, s_old)
%DISTRIBUTEDCONTROLLER Summary of this function goes here
%   Inputs:
%           t: simulation time
%           pos: position vector of every agent
%           vel: velocity vector of every agent
%           sm: sliding manifold variable
%           s_new: Updated discrete signal from algorithm
%           s_old: Old value of discrete signal from algorithm
%   Ouput:
%           u: Desired control signal
%
%   Description:
%           This script computes the new control signal, based on the
%           designed control law of chapter 4. 
    
    global C
    global n
    global N
    global G
    global T
    global M
    global theta
    global maxM
    global t_con
    global u_global
    global ki1 k
    global lamda1
    global lamda2
    % disp("Inside the  controller...");
      
    c_M_i = .01;                         % Mass derivative constant
    di = 0.1;                           % Upper bound of disturbance vector

    % Continuous adjacent agents measurements signals
    b_dot = drho(t - floor(t/T)*T)*(s_new - s_old);
    % disp("");
    % disp("sample rate measurements");
    % disp(b_dot);
    b_ddot = ddrho(t - floor(t/T)*T)*(s_new - s_old);
    % disp("");
    % disp("sample rate measurements");
    % disp(b_ddot);
    
    % Nonlinearities of control law
    y = maxM*norm(theta*vel - (b_ddot + theta*b_dot)) + 0.5*c_M_i*norm(vel)*norm(sm);
    % disp("");
    % disp(y);
    % disp("");
    % disp("Sliding Manifold...");
    % disp(sm);
    
    
    % Robust feedback linearization with L1 properties
    Q = C*vel + G;
    u = Q - ki1*sm - ((y + di)*sm)/(norm(sm,1) + exp(-(lamda1 + lamda2*y)*t));
    % disp("Finished computing control signal...");
    % disp("");
    % disp(u);
    u_global = [u_global, u];
    t_con = [t_con; t];
end

