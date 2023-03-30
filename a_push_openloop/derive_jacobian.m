clear

syms x1 x2 x3 x4 x5 u1 u2 u3;

param.L = eye(3);
param.polygen = [-1 -1 1 1;
                 -1 1 1 -1];
param.m = 1.0;
param.mu = 0.3;
param.g = 9.8;
param.L = diag([param.mu * param.m * param.g, param.mu * param.m * param.g, 3.06 * param.mu * param.m * param.g / 4]);


x = [x1; x2; x3; x4; x5];
u = [u1; u2; u3];

f1 = pushDynamicsEqn(0, x, param, 1, u)
f2 = pushDynamicsEqn(0, x, param, 2, u)

dfdx = jacobian(f1, x)
dfdu = jacobian(f1, u)

% 当方块沿着x轴正方向横向移动时的最优控制以及最优轨迹
% v_star = 10;  % 方块期望速度，随便定一个数字
% L= 2;         % 方块的边长，接触点需要偏移边长的一半
% x_star(t) =  [v_star*t 0 0 v_star*t-L/2 0]';
% u_star(t) = [v_star/(umg), 0, 0]';