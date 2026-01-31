x_obs = [1.5; 2];
% tau = 0.5;
% xobsk = kron(x_obs,eye(N));
xk = 0:0.01:150;
yk = 0:0.01:150;
Ak = [1 0; 0 1];
[X, Y] = meshgrid(xk,yk);
lambda = 0.3;
% x0 = [0; 1];
% %f = @(xk) sum(exp(a .* (xk - x0))) + (lambda/2) * norm(xk - x0)^2;
% f = @(xk) 0.25*norm(xk - x0).^4 - 0.5*tau*norm(xk - x_obs).^2 + 0.5*tau/(1 + norm(xk - x_obs).^2);
% % 
% % % Solve with fminunc
%  opts = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
%  [x_star1, fval, exitflag, output, lambda, grad, hessian] = fmincon(f, x0, [], [], Ak, zeros(2,1),[],[],[],opts);
x0 = [1;0;1;
      1.5;-2;2.4;
      4;1.1;2;
      -0.1;5;0.5;
      3;2.5;3.4];

function f = fk(x)
    % a = 0.1;
    tau = 10.0;
    x_obs = [-1.0; 0.2; 1.0];
    x0 = [1.0 1.5 4.0 -0.1 3.0];
    y0 = [0.0 -2.0 1.1 5.0 2.5];
    z0 = [1.0 2.4 2.0 0.5 3.4];
    s1 = [0; 0; 0];
    s2 = [0; 0; 0];
    for i = 1:5
        s1 = s1 + norm(x - [x0(i); y0(i); z0(i)])^2.*(x - [x0(i); y0(i); z0(i)]); 
        s2 = s2 + (x - [x0(i); y0(i); z0(i)]); 
    end
    f(1) = s1(1) - 5*tau*(x(1) - x_obs(1)) - 5*tau*(x(1) - x_obs(1))/((1 + norm(x - x_obs)^2)^2);
    f(2) = s1(2) - 5*tau*(x(2) - x_obs(2)) - 5*tau*(x(2) - x_obs(2))/((1 + norm(x - x_obs)^2)^2);
    f(3) = s1(3) - 5*tau*(x(3) - x_obs(3)) - 5*tau*(x(3) - x_obs(3))/((1 + norm(x - x_obs)^2)^2);
end
fun = @fk;
x_0 = [7.5; 7.5; 7.5];
x_solve = fsolve(fun,x_0);