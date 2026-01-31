global x_solve
x0 = [1;0;1;
      1.5;-2;2.4;
      4;1.1;2;
      -0.1;5;0.5;
      3;2.5;3.4];
a = 1.5;
tau = 0.5;
function f = f(x)
    a = 0.1;
    tau = 0.0;
    x_obs = [1.5; 2; 1.5];
    x0 = [1 1.5 4 -0.1 3];
    y0 = [0 -2 1.1 5 2.5];
    z0 = [1 2.4 2 0.5 3.4];
    s1 = [0; 0; 0];
    for i = 1:5
        s1 = s1 + norm(x - [x0(i); y0(i); z0(i)])^2.*(x - [x0(i); y0(i); z0(i)]); 
    end
    f(1) = s1(1) - 5*tau*(x(1) - x_obs(1)) - 5*2*tau*(x(1) - x_obs(1))/(1 + norm(x - x_obs)^2)^2;
    f(2) = s1(2) - 5*tau*(x(2) - x_obs(2)) - 5*2*tau*(x(2) - x_obs(2))/(1 + norm(x - x_obs)^2)^2;
    f(3) = s1(3) - 5*tau*(x(3) - x_obs(3)) - 5*2*tau*(x(3) - x_obs(3))/(1 + norm(x - x_obs)^2)^2;
end
fun = @f;
x_0 = [0.7; 0.8; 0.9];
x_solve = fsolve(fun,x_0);