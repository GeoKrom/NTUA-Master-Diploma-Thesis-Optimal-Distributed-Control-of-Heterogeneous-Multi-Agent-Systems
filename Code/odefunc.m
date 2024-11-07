function dstate = odefunc(t, state)
%ODEFUNC Summary of this function goes here
%   Detailed explanation goes here
    global n
    global N
    global T
    global s_i
    global Ds
    global theta
    state = state(:);
    X = reshape(state(1:n*N), [n,N]);
    V = reshape(state((n*N + 1):2*n*N), [n,N]);
    l = 1;
    % error variable
    s_i_old = s_i;
    q = X - rho(t - floor(t/T)*T)*s_i - (1 - rho(t - floor(t/T)*T))*s_i_old;
    % Algorithm
    if floor(t/T) == Ds
        [X_alg, s_i] = regProxGPDA(t, q, X);
        l = l + 1;
    else
        X_alg = X;
    end
    qr = reshape(q,[n*N, 1]);
    x = reshape(X_alg, [N*n,1]);
    v = reshape(V, [N*n, 1]);
    s_r = reshape(s_i, [N*n, 1]);
    s_r_old = reshape(s_i_old, [N*n, 1]);
    q_dot = v - rho(t - floor(t/T)*T)*(s_r - s_r_old);
    omega = q_dot + theta*qr;
    % Control
    
    u = robustFeedbackLinearization(t, x, v, omega, s_i, s_i_old);
    new_state = [x; v];
    dv = lagrangianDynamics(t, new_state, u);
    dstate = [v; dv];
end

