function u = robustFeedbackLinearization(t, pos, vel, sm, s_new, s_old)
%DISTRIBUTEDCONTROLLER Summary of this function goes here
%   Detailed explanation goes here
    global C
    global n
    global N
    global G
    global T
    global M
    global theta
    
    si_now = reshape(s_new, [n*N,1]);
    si_prev = reshape(s_old, [n*N,1]);
    ki1 = 10;
    c_M_i = 0.01;
    di = 0.1;
    b_dot = drho(t - floor(t/T)*T)*(si_now - si_prev);
    b_ddot = ddrho(t - floor(t/T)*T)*(si_now - si_prev);
    y = max(eig(M))*norm(theta*vel - (b_ddot + theta*b_dot)) + 0.5*c_M_i*norm(vel)*norm(sm);
    lamda1 = 0.2;
    lamda2 = 0.4;
    Q = C*vel + G;
    u = Q - ki1*sm - (y + di)*sm/(norm(sm) + exp(-(lamda1 + lamda2*y)*t));
    disp("Inside controller with vector size: ");
    disp(size(u));
end

