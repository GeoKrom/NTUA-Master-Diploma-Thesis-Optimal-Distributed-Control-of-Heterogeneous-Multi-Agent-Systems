
% --- Παράμετροι ---
x0_i = [1; 1];       % επιθυμητή θέση πράκτορα
x_obs = [-1.0; 0.2];      % θέση εμποδίου
alpha_i = 10;         % βάρος "έλξης/άπωσης"
tau = 10;             % βάρος αποφυγής εμποδίου
x0 = [1;0;1;
      1.5;-2;2.4;
      4;1.1;2;
      -0.1;5;0.5;
      3;2.5;3.4];
N = 5;
n = 3;
X0 = reshape(x0, [n, N]);
% --- Πεδίο ορισμού ---
[x1, x2] = meshgrid(-4:0.05:6, -4:0.05:6);

% --- Υπολογισμός της f_i(x) ---
f = zeros(size(x1));
gx = zeros(size(x1)); 
gy = zeros(size(x1));
for i = 1:size(x1,1)
    for j = 1:size(x1,2)
        x = [x1(i,j); x2(i,j)];
        term1 = 0.25 * norm(x - x0_i)^4;
        term2 = -0.5 * alpha_i * norm(x - x_obs)^2;
        term3 = tau / (1 + norm(x - x_obs)^2);
        f(i,j) = term1 + term2 + term3;
        % gradient
        grad = (norm(x - x0_i)^2) * (x - x0_i) ...
             - alpha_i * (x - x_obs) ...
             - (2*tau*(x - x_obs)) / (1 + norm(x - x_obs)^2)^2;

        gx(i,j) = grad(1);
        gy(i,j) = grad(2);
    end
end

% --- 3D επιφάνεια ---
figure(1);
clf;
% colors = lines(N);
surf(x1, x2, f, 'EdgeColor', 'none');
colormap turbo;
hold on;
plot3(x_obs(1),x_obs(2),1.0,'MarkerSize',8,'Color','k','LineWidth',2,'Marker','x')
plot3(x_solve(1),x_solve(2),x_solve(3), 'MarkerSize',8,'Color','r','LineWidth',2,'Marker','x');
% line([x_solve(1) x_solve(1)],[x_solve(2) x_solve(2)],[0 x_solve(3)], 'LineWidth', 4,'Color','k', 'LineStyle','--');
% for i = 1:N
%     plot3(X0(1,i),X0(2,i),X0(3,i), 'MarkerSize',8,'Color',colors(i),'LineWidth',2,'Marker','diamond');
% end
xlabel('$x_1$','FontSize',15,'Interpreter','latex'); ylabel('$x_2$','FontSize',15,'Interpreter','latex'); zlabel('$f_i (x)$','FontSize',15,'Interpreter','latex');
view(45, 30);
legend({'$f_i(x)$','$x_{obs}$','$x^*$'}, ...
       'Location', 'northeast', 'Interpreter','latex');
grid on;

figure(2);
clf;
contourf(x1, x2, f, 30, 'LineColor', 'none');
hold on;
plot(x_solve(1), x_solve(2), 'rx', 'MarkerSize', 10, 'LineWidth', 3); % επιθυμητή θέση
plot(x_obs(1), x_obs(2), 'kx', 'MarkerSize', 10, 'LineWidth', 3); % εμπόδιο
colormap turbo;
xlabel('$x_1$','FontSize',15,'Interpreter','latex'); ylabel('$x_2$','FontSize',15,'Interpreter','latex');
axis equal;
grid on;
legend({'Contours $f_i(x)$', '$x^*$', '$x_{obs}$'}, ...
       'Location', 'bestoutside', 'Interpreter','latex');