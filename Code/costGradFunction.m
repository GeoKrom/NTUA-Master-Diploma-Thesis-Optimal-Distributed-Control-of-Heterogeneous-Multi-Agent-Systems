function fi = costGradFunction(x, x_init, a_i)
%COSTFUNCTION returns a Nonconvex function
%       Inputs:
%               x: position of every agent
%               x_init: initial position of every agent
%               a_i: parameter vector
%       Output:
%               fi: Computed gradient of cost function
%
%       Description:
%               The gradient of every cost function f_i is computed to be
%               used on the ProxGPDA.

global tau
global x_obs
global n
global N
global X_f

% Method 1 - Simple Non-Comvex function
% fi = exp(-a_i*x)./(1 + exp(-a_i*x)).^2;
% fi = f_i(:,2);

% Method 2 - Obstacle Avoidance
 fi = (norm(x - x_init) - (tau/(tau + 1))*norm(x - x_obs));

% Method 3 - Formation Control with Obstavle Avoidance
% x_f = reshape(X_f, [n*N, 1]);
% fi = (norm(x + X_f - x_init) + (tau/(tau + 1))*norm(x + X_f - x_obs))*ones(n,1);


disp("Cost function vector");
disp(fi);
assignin("base","fi",fi);
end

