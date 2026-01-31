clear all;
clc;

global Lplus beta x_obs tau n N x0 x_j_sum D
beta = 200;
n = 3;
N = 5;
x_obs = [-1.0; 0.2; 1.0];
tau = 10;
x0 = [1;0;1;
      1.5;-2;2.4;
      4;1.1;2;
      -0.1;5;0.5;
      3;2.5;3.4];
x0_ = reshape(x0, [n,N]);
D = diag([2 3 1 3 1]);
x_old = x0_;
x_alg = [];
x_new = zeros(n,N);
Lplus = [2 1 0 1 0;
        1 3 0 1 1;
        0 0 1 1 0;
        1 1 1 3 0;
        0 1 0 0 1];
iterations = 1500;
params.lambda = 0.3;
params.sigma = 1.0;
params.b = pi / 3;
params.delta = 1e-2;
params.epsilon = 1e-2;
params.xobs = x_obs;
x_j_sum = zeros(n,N);
c1 = 200;
c2 = 0.001;
a = 0.9;      % exponential decay rate

for i = 1:iterations
    dist_mag = (c1 - c2) * a^iterations + c2;
    disturbance = dist_mag*rand(n,N);
    x_new = regProxGPDA(x_old) + disturbance;

    x_old = x_new; %reshape(x_new, [n*N,1]);
    x_alg = [x_alg; x_new];
end

% for i = 1:3:size(x,1)
% 
% end