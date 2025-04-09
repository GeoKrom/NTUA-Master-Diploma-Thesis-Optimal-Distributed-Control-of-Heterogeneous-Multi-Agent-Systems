function [x, s] = regProxGPDA(t, q_in, x_in, mu_in)
%PROXGPDA Summary of this function goes here
%   Inputs:
%           t: simulation time
%           q_in: position error vector of every agent
%           x_in: position vector of every agent
%   Ouputs:
%           x: Updated position from algorithm
%           s: Updated discrete signal
%
%   Description:
%           This script update the primal variable which is based on ProxGPDA, based on
%           chapter 3.

global T
global D
global beta
global N
global n
global epsilon
global D
global Lplus
global A
global Ds
global x0
global mu
global ai
global method
global s_i xinit s_i_old
global s_val
nu = 1;
ell = 1;
df = zeros(n,1);
x_i = zeros(n,N);
si = zeros(n,1);
xo = zeros(n,1);
x_0 = reshape(x0, [n,N]);
xin = reshape(x_in(1,1:n*N),[n,N]);
disp(xin);
mu_new = reshape(mu_in, [n,N]);
% Main loop of algorithm
for ii = 1:N
    disp("");
    disp("Agent ");
    disp(ii);
    disp("");
    x_j_i = zeros(n,1);
    x_j_sum = zeros(n,1);
    gradFi = costGradFunction(xin(:,ii), x_0(:,ii), ai(ii));
    for jj = 1:N
        % Check connectivity of every agent to update variables
        if (abs(A(ii,jj)) == 1) && ii ~= jj
            x_j_i = xin(:,jj) + x_j_i;
            x_j_sum = x_j_sum + mu_new(:,ii);
            % Primal variable update
            si = xin(:,ii) - 1/(2*beta*D(ii,ii))*(beta*x_j_i - gradFi  - beta*x_j_sum);
            xo = si + q_in(:,ii);
        end
    end
    s_i(:,ii) = si;
    x_i(:,ii) = xo;
end
% end
x = x_i;
s = s_i;
s_val = [s;s_val];
disp("New position");
disp("");
disp(x);
end

