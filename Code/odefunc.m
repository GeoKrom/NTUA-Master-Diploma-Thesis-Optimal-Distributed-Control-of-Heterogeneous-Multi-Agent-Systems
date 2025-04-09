function dstate = odefunc(t, state)
%ODEFUNC Summary of this function goes here
%   Detailed explanation goes here
global n
global N
global T
global s_i
global Ds
global theta
global x0
global q_i
global t_err
global q_res
global X_alg
global method
global k
global X_f
global q v s_i_old s_i_new
% disp("Start computing for all agents...");


% Change vector n*Nx1 to a matrix nxN
X = reshape(state(1:n*N), [n,N]);
V = reshape(state((n*N + 1):2*n*N), [n,N]);
x = reshape(X, [N*n,1]);
% error variables
% Keep old value of si to compute error

q = X - rho(t - floor(t/T)*T)*s_i - (1 - rho(t - floor(t/T)*T))*s_i_old;

% Formation Control
 % q = (X - X_f) - rho(t - floor(t/T)*T)*s_i - (1 - rho(t - floor(t/T)*T))*s_i_old;
% Keep every norm update for visualization
for i = 1:N
    q_res(i) = norm(q(:,i));
end
q_i = [q_i, q_res'];
t_err = [t_err, t];
% disp("");
% disp("Error variables...")
% disp(q);

% Sliding manifold computation
qr = reshape(q,[n*N, 1]);
v = reshape(V, [N*n, 1]);
s_r = reshape(s_i, [N*n, 1]);
s_r_old = reshape(s_i_old, [N*n, 1]);
q_dot = v - rho(t - floor(t/T)*T)*(s_r - s_r_old);
omega = q_dot + theta*qr;

% Control Signal
u = robustFeedbackLinearization(t, x, v, omega, s_r, s_r_old);

% New state after algorithm computation
new_state = [x; v];

% Agents dynamics
dv = lagrangianDynamics(t, new_state, u);
dstate = [v; dv];

end

