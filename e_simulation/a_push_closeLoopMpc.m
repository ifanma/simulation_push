yalmip('clear');
clear;

%% basic parameter
param.l = 0.05;
param.polygen = param.l *[-1 -1 1 1;
                          -1 1 1 -1];
param.m = 0.3;
param.mu = 0.3;         % 地面摩擦
param.mu_c = 0.3;       % 操作摩擦
param.g = 9.81;
umg = param.mu * param.m * param.g;
param.L = diag([2/umg^2, 2/umg^2,...
                2/(0.03825* umg)^2]);

derive_jacobian(param);

param.simdt = 0.001;    % 仿真器周期
param.ctrldt = 0.01;    % 控制器周期
param.predt = 0.05;     % 参考轨迹周期
param.plotdt = 0.05;    % 绘图周期
param.v_star = 0.05;

param.z_rec = [];
param.u_rec = [];
param.x_rec = [];


%% mpc parameter
param.N = 15;
N = param.N;
param.traj = @nominalTraj;    param.tf = 1.5; 
% param.traj = @nominalTraj2;   param.tf = 5.0; 
% param.traj = @nominalTraj3;   param.tf = 5.0; 
    
% Model data
A = sdpvar(repmat(5,1,N), repmat(5,1,N), 'full');
B = sdpvar(repmat(5,1,N), repmat(3,1,N), 'full');
nx = 5; % Number of states
nu = 3; % Number of inputs

% MPC objective
Q = 5 * diag([1, 1, 4, 0.1, 2]);    % 状态跟踪代价
R = 3 * diag([1, 4, 0.1]);       % 控制跟踪代价
W = 0.2 * diag([0.1, 1, 1]);        % 状态先验代价，认为不滑动的状态较好
V = 0.8 * diag([1, 1, 1]);          % 状态切换代价，认为不切换最好


%% build model
u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
u_star = sdpvar(repmat(nu,1,N), repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x_star = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x0 = sdpvar(nx,1);
z = binvar(repmat(3,1,N), repmat(1,1,N));

xl = [-10, -10, -10, -param.l, -param.l *0.9]';
xu = [10, 10, 10, -param.l , param.l *0.9]';
ul = [0, -10, -10]';
uu = [10, 10, 10]';

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
x0_ = [0, param.l, pi/6, -param.l, param.l/2]';
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
plot(ind)

figure(3)
plot(param.u_rec')
legend('nc', 'tc', 'phi')

figure(4)
clf
plot(param.x_rec(1:3, :)', '*')
legend('x', 'y', '\theta');
hold on
xref = [];
for i = 1:size(param.x_rec,2)
    xref(:, end+1) = param.traj((i-1)*param.simdt, param);
end
plot(xref(1:3, :)')

