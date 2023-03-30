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
% 0.03825
derive_jacobian(param);

param.simdt = 0.001;    % 仿真器周期
param.ctrldt = 0.01;    % 控制器周期
param.predt = 0.02;     % 参考轨迹周期
param.plotdt = param.predt;    % 绘图周期
param.v_star = param.l * 5;

param.z_rec = [];
param.u_rec = [];
param.x_rec = [];

%% mpc parameter
param.N = 50;
N = param.N;
param.traj = @nominalTraj;    param.tf = 20; 
% param.traj = @nominalTraj2;   param.tf = 5.0; 
% param.traj = @nominalTraj3;   param.tf = 5.0; 
% param.traj = @nominalTraj4;   param.tf = 5.0; 
    
% MPC objective
Q = 2 * diag([3, 3, 1, 0, 5]);    % 状态跟踪代价
R = 1 * diag([5, 5, 1]);       % 控制跟踪代价
W = 0.2 * diag([0.5, 1, 1]);        % 状态先验代价，认为不滑动的状态较好
V = 0.2 * diag([1, 1, 1]);          % 状态切换代价，认为不切换最好

d = 0.004;
xl = [-10, -10, -10, -param.l - 0.0005 - d, -param.l *0.9]';
xu = [10, 10, 10, -param.l - 0.0005 + d, param.l*0.9]';
ul = [0, -10, -10]';
uu = [1, 10, 10]';

%% build model
A = sdpvar(repmat(5,1,N), repmat(5,1,N), 'full');
B = sdpvar(repmat(5,1,N), repmat(3,1,N), 'full');
nx = 5; % Number of states
nu = 3; % Number of inputs

u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
u_star = sdpvar(repmat(nu,1,N), repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x_star = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x0 = sdpvar(nx, 1);
z = binvar(repmat(3,1,N), repmat(1,1,N));

constraints = [];
objective   = 0;
constraints = [constraints, x{1} == x0] ;
for k = 1:N
    objective = objective +(x{k}-x_star{k})'*Q*(x{k}-x_star{k}) + (u{k}-u_star{k})'*R*(u{k}-u_star{k}) + z{k}'*W*z{k};
    if k ~= 1
         objective = objective + (z{k}-z{k-1})'*V*(z{k}-z{k-1});
    end
    Mode1 = [u{k}(3) == 0, -param.mu_c*u{k}(1) <=u{k}(2)<= param.mu_c*u{k}(1)];
    Mode2 = [u{k}(3) >= 0, u{k}(2) == param.mu_c * u{k}(1)];
    Mode3 = [u{k}(3) <= 0, u{k}(2) == -param.mu_c * u{k}(1)];
    constraints = [constraints, implies(z{k}(1), Mode1), implies(z{k}(2), Mode2), implies(z{k}(3), Mode3)];
    constraints = [constraints, sum(z{k}) == 1];
    constraints = [constraints, x{k+1}- x_star{k+1} == A{k}*(x{k}-x_star{k}) + B{k}*(u{k}-u_star{k})] ;
    constraints = [constraints, ul<= u{k}<= uu, xl<= x{k}<= xu];
end
objective = objective + (x{k+1}-x_star{k+1})'*Q*(x{k+1}-x_star{k+1});
constraints = [constraints, xl<= x{k+1}<= xu];

parameters_in = {x0, [A{:}], [B{:}], [u_star{:}], [x_star{:}]};
solutions_out = {[u{:}], [x{:}], [z{:}]};
ops = sdpsettings('verbose',2, 'solver','gurobi', 'savedebug', '1');
controller = optimizer(constraints, objective,ops,parameters_in,solutions_out);
param.controller = controller;


%% dynamics close-loop
% x0, y0, theta0, contact_x, contact_y
% x0 = [0, 0, pi/6, -1, 0.5]';
% % x0 = [0, 0, 0, -1, 0]';
% [x_sim, param] = RungeKutta4th(@pushDynamicsEqn, x0, param, param.simdt, 0.8);


%% one shoot
% x0, y0, theta0, contact_x, contact_y
x0_ = [0, param.l, pi/6, -param.l, param.l/2]';
% x0_ = [0, 0, pi/6, -param.l, param.l/2]';
% x0_ = [0, 0, 0, -param.l, 0]';

[~, param] = controlEqn(0, x0_, param);

t_sim = 0: param.predt: param.N * param.predt;
x_rec = param.x_rec_oneshoot;
u_rec = param.u_rec_oneshoot;
z_rec = param.z_rec_oneshoot;


%% draw figures
plot_state(t_sim, x_rec, param);    

figure(2)
[~, ind] = max(z_rec);
plot(ind)

figure(3)
plot(u_rec')
legend('nc', 'tc', 'phi')

figure(4)
clf
plot(x_rec', '*')
legend('x', 'y', '\theta', 'n', 't');
hold on
xref = [];
for i = 1:size(x_rec,2)
    xref(:, end+1) = param.traj((i-1)*param.predt, param);
end
plot(xref')

