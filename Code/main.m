clear;
clc;
tic;

%% Main program of simulation
%   odefunc.m: script of ode function
%   regProxGPDA.m: Gradient Proximal PDA algoritm script
%   robustFeedbackLinearization: Controller script
%   costGradFunction.m: Gradient of cost function script
%   lagrangianDynamics.m: Agent's dynamics script
%   rho.m: Zero Hold Order function script
%   drho.m: Gradient of Zero Hold Order script
%   ddrho.m: Double gradient of Zero Hold Order script

% Usage: main | In the command window. Every script is called inside other
%               scripts.

%% Global variables
global T
global D
global beta
global N
global k
global n
global epsilon
global Lplus
global q_i
global A
global Ds
global x_obs
global M
global tau
global mu
global x0
global fi
global ai
global v_dot
global s_i
global d_max
global theta
global G
global C
global Mk
global d
global maxM
global X_alg
global u_global
global t_con
global t_err
global ki1
global lamda1
global lamda2
global q_res
global z_f
global R
global s_val
global X_f
global method
global q v s_i_old triggerTime s_i_new
%% Parameter Initialization
disp("Initialize parameters....");

N = 5;              % Number of agents
n = 3;              % Agent's Dimensions
g = 9.81;
R = 4;            % Formation Radius
z_f = 2;            % Desired Altitude of Formation
T = 2;              % Sampled period time
delta = 2;          % Algorithm constant
L = 2;              % Lipschitz constant

Ds = 6;             % Ds = kT samples
d_max = 0.4;        % ZOH delay factor
fi = zeros(n,1);
v_dot = zeros(N*n,1);
s_i = zeros(n,N);
s_i_old = zeros(n,N);
theta = 0.5*eye(n*N);  % Sliding manifold Gains
k = 1;
s_val = [];
%% Use your desired method for consensus
% method = "Formation Control";
method = "Optimal Consensus";
ai = [2.4; 1.3; 0.5; 0.84; 1.25];   % Logistic function constants per agent
mu = zeros(N*n,1);
% Mass matrix
M = [2.1 0 0 0 0;
    0 3.2 0 0 0;
    0 0 2 0 0;
    0 0 0 1.8 0;
    0 0 0 0 2.8];
% Incidence matrix of graph
A = [1 -1 0 0 0;
    1 0 0 -1 0;
    0 1 0 -1 0;
    0 1 0 0 -1;
    0 0 1 -1 0];
% Signless Laplacian matrix
Lplus = [2 1 0 1 0;
    1 3 0 1 1;
    0 0 1 1 0;
    1 1 1 3 0;
    0 1 0 0 1];
% Signed Laplacian matrix
Lsigned = [2 -1 0 -1 0;
    -1 3 0 -1 -1;
    0 0 1 -1 0;
    -1 -1 -1 3 0;
    0 -1 0 0 1];
% Degree matrix
D = diag([2,3,1,3,1]);
maxM = max(eig(M));

g1 = [0;0;g];
G1 = reshape(g1.*ones(1,n*N), [n*N,3]);
G = G1(:,1);                                   % Gravity vector for all agents
C = kron(diag([0.2,0.1,0.15,0.58,0.42]),eye(n));
Ak = kron(A,eye(n));
Lk = kron(Lsigned,eye(n));
Dk = kron(D, eye(n));
Mk = kron(M, eye(n));
% ProxGPDA algorithm constants. Desired values from Corollary 3.1
ep = 1e-4;
Eig = eig(Lsigned);
minNonZeroEig = min(Eig(Eig > 1e-10));
c = max(delta/L, 6*norm(Lplus)/minNonZeroEig);
beta = 0.5*(2*L*(2+c) + (1+c)*ep + sqrt((2*L*(2+c) + (1+c)*ep)^2 + 24*L^2/minNonZeroEig)) + ep;

% Obstacle Avoidance cost function
x_obs = [1.0; 0.0; 0.5];
tau = 0.6;
X_f = zeros(n,N);
for i = 1:N
    X_f(:,i) =  R*[cos(i*(pi/3)); sin(i*(pi/3)); z_f];
end

disp("Initialize state...");
dt = 1e-4;
tspan = 0:dt:120;
tstart = 0;
tend = 30;
x0 = [1;0;1;
    1.5;-2;2.4;
    4;1;2;
    -0.1;5;0.5;
    3;2.5;3.4];
X_alg = zeros(n,N);

X0 = reshape(x0,[n,N]);
sum = zeros(n,1);
for i = 1:N
    sum = sum + X0(:,i);
end
x_star = zeros(n,1);
x_star = 1/N*sum + tau*(1/N*sum - x_obs);
xinit = [x0; zeros(n*N, 1)];


%% Control Parameter Gains
% Static control gain
ki1 = 10;
% Exponential gain parameters
lamda1 = .055;
lamda2 = .015;

%% Ode simulation
u_global = [];
q_i = [];
t_con = [];
t_err = [];
q_res = zeros(size(N));
t = [];
x = [];
tout = [];
xout = [];
ell = 1;
%% TODO
% Change the mapping of ODE.
% regProxPDA outside of ode.
% Solving the ode recursivly in interval [kT, (k+1)T].
while tstart < tend
    s_i_old = s_i;

    opt = odeset('RelTol',1e-13,'AbsTol',1e-9,'Events',@(tout, xout) event(tout, xout));
    [tout, xout] = ode45(@(tout, xout) odefunc(tout, xout), tstart:dt:tend, xinit, opt);
    tstart = tout(end);
    [x_alg, s_i] = regProxGPDA(tout, q, xout, mu);
    x_alg = reshape(x_alg,[n*N,1]);
    xinit = [x_alg;v];
    % Dual variable update
    if ell < Ds
        ell = ell + 1;
    else
        mu = mu + Ak*xout(1,1:n*N)';
        % disp(mu);
        ell = 1;
    end
   
    t = [tout;t];
    x = [xout;x];
    k = k + 1;
    disp("");
    disp("Number iteration");
    disp(k);
end
disp("End of Simulation...");
% xnew = [];
% tnew = [];
% for i = 1:size(x,1)
%     if x(i,:) == x(i+1,:)
%         continue;
%     else
%         xnew(i,:) = x(i,:);
%         tnew(i,:) = t(i,:);
%     end
% 
% end

%% Plots
t1 = t_con(end/2:end);
figure(1);
clf;
plot(t,x(:,1),'r-');
hold on;
plot(t,x(:,4),'m-');
plot(t,x(:,7),'g-');
plot(t,x(:,10),'k-');
plot(t,x(:,13),'y-');
plot(t,x_star(1).*ones(size(t)),'b--');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter','latex');
ylabel('$x_{i1} [m]$', 'FontSize', 14, 'Interpreter','latex');
legend("$x_1$","$x_2$","$x_3$","$x_4$","$x_5$","$x^*$","Interpreter","latex",'Orientation','horizontal');

figure(2);
clf;
plot(t,x(:,2),'r-');
hold on;
plot(t,x(:,5),'m-');
plot(t,x(:,8),'g-');
plot(t,x(:,11),'k-');
plot(t,x(:,14),'y-');
plot(t,x_star(2).*ones(size(t)),'b--');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter','latex');
ylabel('$x_{i2} [m]$', 'FontSize', 14, 'Interpreter','latex');
legend("$y_1$","$y_2$","$y_3$","$y_4$","$y_5$","$y^*$","Interpreter","latex",'Orientation','horizontal');

figure(3);
clf;
plot(t,x(:,3),'r-');
hold on;
plot(t,x(:,6),'m-');
plot(t,x(:,9),'g-');
plot(t,x(:,12),'k-');
plot(t,x(:,15),'y-');
plot(t,x_star(3).*ones(size(t)),'b--');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$x_{i3} [m]$', 'FontSize', 14, 'Interpreter','latex');
legend("$z_1$","$z_2$","$z_3$","$z_4$","$z_5$","$z^*$","Interpreter","latex",'Orientation','horizontal');


figure(4);
clf;
plot(t_con,u_global(1,:),'r-');
hold on;
plot(t_con,u_global(4,:),'m-');
plot(t_con,u_global(7,:),'g-');
plot(t_con,u_global(10,:),'k-');
plot(t_con,u_global(13,:),'b-');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter','latex');
ylabel('$u_{i1}$', 'FontSize', 14, 'Interpreter','latex');
legend("$u_1$","$u_2$","$u_3$","$u_4$","$u_5$","Interpreter","latex","Location","southeast",'Orientation','horizontal');

axes('Position',[.35 .35 .4 .2]);
box on;
grid on;
hold on
plot(t1,u_global(1,end/2:end),'r-');
plot(t1,u_global(4,end/2:end),'m-');
plot(t1,u_global(7,end/2:end),'g-');
plot(t1,u_global(10,end/2:end),'k-');
plot(t1,u_global(13,end/2:end),'b-');



figure(5);
clf;
plot(t_con,u_global(2,:),'r-');
hold on;
plot(t_con,u_global(5,:),'m-');
plot(t_con,u_global(8,:),'g-');
plot(t_con,u_global(11,:),'k-');
plot(t_con,u_global(14,:),'b-');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter','latex');
ylabel('$u_{i2}$', 'FontSize', 14, 'Interpreter','latex');
legend("$u_1$","$u_2$","$u_3$","$u_4$","$u_5$","Interpreter","latex","Location","southeast",'Orientation','horizontal');
axes('Position',[.35 .35 .4 .2]);
box on;
grid on;
hold on
plot(t1,u_global(2,end/2:end),'r-');
plot(t1,u_global(5,end/2:end),'m-');
plot(t1,u_global(8,end/2:end),'g-');
plot(t1,u_global(11,end/2:end),'k-');
plot(t1,u_global(14,end/2:end),'b-');

figure(6);
clf;
plot(t_con,u_global(3,:),'r-');
hold on;
plot(t_con,u_global(6,:),'m-');
plot(t_con,u_global(9,:),'g-');
plot(t_con,u_global(12,:),'k-');
plot(t_con,u_global(15,:),'b-');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$u_{i3}$', 'FontSize', 14, 'Interpreter','latex');
legend("$u_1$","$u_2$","$u_3$","$u_4$","$u_5$","Interpreter","latex","Location","southeast",'Orientation','horizontal');
axes('Position',[.35 .35 .4 .2]);
box on;
grid on;
hold on
plot(t1,u_global(3,end/2:end),'r-');
plot(t1,u_global(6,end/2:end),'m-');
plot(t1,u_global(9,end/2:end),'g-');
plot(t1,u_global(12,end/2:end),'k-');
plot(t1,u_global(15,end/2:end),'b-');

figure(7);
clf;
plot(t_err, q_i(1,:),'r-');
hold on;
plot(t_con,q_i(2,:),'m-');
plot(t_con,q_i(3,:),'g-');
plot(t_con,q_i(4,:),'k-');
plot(t_con,q_i(5,:),'b-');
hold off;
grid on;
xlabel('$time [s]$', 'FontSize', 14, 'Interpreter','latex');
ylabel('$\|q_i\|$', 'FontSize', 14, 'Interpreter','latex');
legend("$\|q_1\|$","$\|q_2\|$","$\|q_3\|$","$\|q_4\|$","$\|q_5\|$","Interpreter","latex",'Orientation','horizontal');
axes('Position',[.35 .35 .4 .2]);
box on;
grid on;
hold on
plot(t_err(end/2:end),q_i(1,end/2:end),'r-');
plot(t_err(end/2:end),q_i(2,end/2:end),'m-');
plot(t_err(end/2:end),q_i(3,end/2:end),'g-');
plot(t_err(end/2:end),q_i(4,end/2:end),'k-');
plot(t_err(end/2:end),q_i(5,end/2:end),'b-');

% figure(8);
% clf;
% axis([-10 10 -10 10 0 10]); 
% [x,y] = meshgrid(-10:1:10,-10:0.1:10);
% z = 0*x + 0*y;
% surf(x, y, z);
% xlabel("$x [m]$","Interpreter","latex");
% ylabel("$y [m]$","Interpreter","latex");
% zlabel("$z [m]$","Interpreter","latex");



