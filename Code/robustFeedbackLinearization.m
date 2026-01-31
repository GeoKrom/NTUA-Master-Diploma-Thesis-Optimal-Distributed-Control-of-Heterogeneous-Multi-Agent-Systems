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
    
    global C C1
    global n
    global N
    global G
    global T
    global M
    global theta
    global maxM
    global t_con
    global u_global
    global ki1 g1 ki2
    global lamda1
    global lamda2 k q
    % disp("Inside the  controller...");
      
    c_M_i = 0.0;                       % Mass derivative constant
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
    sm_mat = reshape(sm, [n,N]);
    yi = zeros(N,1);
    b_dot_mat = reshape(b_dot,[n,N]);
    b_ddot_mat = reshape(b_ddot,[n,N]);
    vel_mat = reshape(vel, [n,N]);

    for i = 1:N
        yi(i) = maxM*norm(theta*vel_mat(:,i) - (b_ddot_mat(:,i) + theta*b_dot_mat(:,i))) + 0.5*c_M_i*norm(vel_mat(:,i))*norm(sm_mat(:,i));
    end
    % disp(theta);
    % disp(yi);
    % Nonlinearities of control law
    % y = maxM*norm(theta*vel - (b_ddot + theta*b_dot)) + 0.5*c_M_i*norm(vel)*norm(sm);
    % disp("");
    % disp(y);
    % disp("");
    % disp("Sliding Manifold...");
    % disp(sm);
    
    % Smooth denominator
    % norm_sm = norm(sm, 1);
    norm_sm_i = zeros(N,1);
    for i = 1:N
        norm_sm_i(i) = norm(sm_mat(:,i));
    end
    u_mat = zeros(n,N);
    for i = 1:N
        % u_mat(:,i) = -ki1*sm_mat(:,i);
        u_mat(:,i) = C1(i,i)*vel_mat(:,i) + g1 - ki1*sm_mat(:,i) - ((yi(i) + di + ki2)*sm_mat(:,i))/(norm_sm_i(i) + exp(-(lamda1 + lamda2*yi(i))*t));
    end
    % denominator = norm_sm + exp(-(lamda1 + lamda2*y)*t);
    % Q = C*vel + G;
    % Final control law
    % u = Q - ki1*sm - ((y + di) * sm)/denominator;
    % Robust feedback linearization with L1 properties
    u = reshape(u_mat, [n*N,1]);
    % u = -ki1*(pos);
    %u = ki1*sm - (y + di)*sm/(norm(sm,1) + exp(-(lamda1 + lamda2*y)*t));
    if any(isnan(u)) || any(isinf(u)) || any(abs(u) > 1e6)
        error("Control input u unstable at t = %.4f (max |u| = %.2e)", t, max(abs(u)));
    end
    % disp("Finished computing control signal...");
    % disp("");
    % disp(u);
    u_global = [u_global, u];
    t_con = [t_con; t];
end

