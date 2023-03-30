yalmip('clear');
clear;

%% basic parameter
param.polygen = [-1 -1 1 1;
                 -1 1 1 -1];
param.m = 1.0;
param.mu = 0.3;         % 地面摩擦
param.mu_c = 0.3;       % 操作摩擦
param.g = 9.8;
param.L = diag([param.mu * param.m * param.g, param.mu * param.m * param.g, 3.06 * param.mu * param.m * param.g / 4]);
% integral_(-1)^1 integral_(-1)^1 sqrt(x^2 + y^2)dx dy = 4/3 (sqrt(2) + sinh^(-1)(1))≈3.06078

param.simdt = 0.001;    % 仿真器周期
param.ctrldt = 0.01;    % 控制器周期
param.predt = 0.05;     % 参考轨迹周期
param.plotdt = 0.05;    % 绘图周期
param.v_star = 5;

param.z_rec = [];
param.u_rec = [];
param.x_rec = [];


%% mpc parameter
param.N = 35;
N = param.N;
% param.traj = @nominalTraj;    param.tf = 1.5; 
% param.traj = @nominalTraj2;   param.tf = 5.0; 
% param.traj = @nominalTraj5;   param.tf = 5.0; 
param.traj = @nominalTraj3;   param.tf = 5.0; 
    
% Model data
A = sdpvar(repmat(5,1,N), repmat(5,1,N), 'full');
B = sdpvar(repmat(5,1,N), repmat(3,1,N), 'full');
nx = 5; % Number of states
nu = 3; % Number of inputs

% MPC objective
Q = 5 * diag([1, 1, 4, 0.1, 2]);    % 状态跟踪代价
R = 3 * diag([0.1, 10, 0.1]);       % 控制跟踪代价
W = 0.2 * diag([0.1, 1, 1]);        % 状态先验代价，认为不滑动的状态较好
V = 0.8 * diag([1, 1, 1]);          % 状态切换代价，认为不切换最好


%% build model
u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
u_star = sdpvar(repmat(nu,1,N), repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x_star = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x0 = sdpvar(nx,1);
z = binvar(repmat(3,1,N), repmat(1,1,N));

xl = [-50, -50, -50, -1, -1]';
xu = [50, 50, 50, -1, 1]';
ul = [0, -50, -50]';
uu = [50, 50, 50]';

constraints = [];
objective   = 0;
constraints = [constraints, x{1} == x0];
for k = 1:N
    objective = objective +(x{k}-x_star{k})'*Q*(x{k}-x_star{k}) + (u{k}-u_star{k})'*R*(u{k}-u_star{k}) + z{k}' * W * z{k};
    if k ~= 1
         objective = objective + (z{k} - z{k-1})' * V * (z{k} - z{k-1});
    end

    Mode1 = [u{k}(3) == 0, -param.mu_c * u{k}(1) <=u{k}(2)<= param.mu_c * u{k}(1)];
    Mode2 = [u{k}(3) >= 0, u{k}(2) == param.mu_c * u{k}(1)];
    Mode3 = [u{k}(3) <= 0, u{k}(2) == -param.mu_c * u{k}(1)];
    
    constraints = [constraints, implies(z{k}(1), Mode1), implies(z{k}(2), Mode2), implies(z{k}(3), Mode3)];
    constraints = [constraints, sum(z{k}) == 1];
    constraints = [constraints, x{k+1}- x_star{k+1} == A{k}*(x{k}-x_star{k}) + B{k}*(u{k}-u_star{k})] ;
    constraints = [constraints, ul<= u{k}<= uu, xl<= x{k}<= xu];
end
objective = objective + (x{k+1}- x_star{k+1})'*Q*(x{k+1}- x_star{k+1});
constraints = [constraints, xl<= x{k+1}<= xu];

parameters_in = {x0, [A{:}], [B{:}], [u_star{:}], [x_star{:}]};
solutions_out = {[u{:}], [x{:}], [z{:}]};
ops = sdpsettings('verbose',2, 'solver','gurobi');
controller = optimizer(constraints, objective,ops,parameters_in,solutions_out);
param.controller = controller;


%% dynamics close-loop
% x0, y0, theta0, contact_x, contact_y
x0_ = [0, 1, -pi/6, -1, 0.5]';
% x0 = [0, 0, 0, -1, 0]';
tf = param.tf;
dt = param.simdt;
tic
[x_sim, param] = RungeKutta4th(@pushDynamicsEqn, x0_, param, dt, tf);
time_spent = toc;
disp(['total time cost: ' num2str(time_spent) ' s'])
t_sim = 0:dt:tf+dt;


%% draw figures
plot_state(t_sim, x_sim, param);    

figure(2)
[~, ind] = max(param.z_rec);

ind(ind == 1) = 0;
ind(ind == 2) = 1;
ind(ind == 3) = -1;
imagesc(flipud(ind)); axis tight; grid off; 
caxis([-1 1]); 
colormap([0 0 1;0 1 0;1 1 0])
colorbar
title('接触状态')

figure(3)
plot(param.u_rec')
legend('nc', 'tc', 'phi')

figure(4)
clf
plot(param.x_rec(1:3, :)', '*')
hold on
xref = [];
for i = 1:size(param.x_rec,2)
    xref(:, end+1) = param.traj((i-1)*param.simdt, param);
end
plot(xref(1:3, :)')
legend('x', 'y', '\theta', 'x_{ref}', 'y_{ref}', '\theta_{ref}');

