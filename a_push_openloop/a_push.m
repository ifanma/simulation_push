param.L = eye(3);
param.polygen = [-1 -1 1 1;
                 -1 1 1 -1];
param.m = 1.0;
param.mu = 0.3;
param.g = 9.8;
param.L = diag([param.mu * param.m * param.g, param.mu * param.m * param.g, 3.06 * param.mu * param.m * param.g / 4]);
% integral_(-1)^1 integral_(-1)^1 sqrt(x^2 + y^2)dx dy = 4/3 (sqrt(2) + sinh^(-1)(1))≈3.06078

% x0, y0, theta0, contact_x, contact_y
x0 = [0, 0, pi/6, -1, 0.5]';
% x0 = [0, 0, 0, -1, 0]';
x = RungeKutta4th(@pushDynamicsEqn, x0, param, 0.001, 0.4);

plot_state(x, param);                                  