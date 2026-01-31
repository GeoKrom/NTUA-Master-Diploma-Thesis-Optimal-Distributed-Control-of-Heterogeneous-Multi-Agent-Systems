function s = regProxGPDA(x_in)
% regProxGPDA: Proximal Gradient Primal-Dual Algorithm update
%
% Inputs:
%   t:     current time (not used)
%   q_in:  tracking error q_i(t) from the controller
%   x_in:  current agent positions (flattened)
%   mu_in: dual variable vector (flattened)
%
% Outputs:
%   x: updated agent positions
%   s: updated auxiliary primal variable s_i

global T D Ds beta N n A xinit mu ai s_val x_obs k s_i x_j_sum x0 X_f x_star1 x_star2 Lplus x_j_i x_solve
x_s = [x_solve x_solve x_solve x_solve x_solve];
% Reshape inputs
x_0 = reshape(x0, [n, N]);      % Initial positions
xin = reshape(x_in, [n, N]);    % Current positions from ODE
% s = zeros(n,N);
params.a = 1.5;
params.lambda = 0.3;
params.sigma = 1.0;
params.b = pi / 3;
params.delta = 1e-2;
params.epsilon = 1e-2;
params.xobs = x_obs;
type = 'nonconobs';
% Loop over agents

for ii = 1:N

    x_j_i = zeros(n,1);
    
    gradFi = costGradFunction(xin(:,ii), x_0(:,ii), params, type);
    
    for jj = 1:N
        if abs(Lplus(ii,jj)) == 1 && ii ~= jj
            x_j_i = x_j_i + xin(:,jj);
            x_j_sum(:,ii) = x_j_sum(:,ii) + (xin(:,ii) - xin(:,jj));
        end
    end
       
    % Primal variable update (from theory)
    s(:,ii) = 0.5*(x_in(:,ii)) + (1/(2*beta*D(ii,ii)))*(beta*x_j_i - gradFi - beta*x_j_sum(:,ii));
    % s(:,ii) = -0.5*x_in(:,ii) - (1/(2 * beta * D(ii,ii))) *(gradFi - gradFi_prev) + 1/D(ii,ii)*x_j_i - 0.5*(1/D(ii,ii)*x_j_i_prev + xin_prev(:,ii));
end
% Update global s_i for next step
% fprintf("‣ Updated dual norm: %.4f\n", norm(x_j_sum));
% disp(x_j_sum);
s_val = [s_val; s];

end