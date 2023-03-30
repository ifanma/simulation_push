yalmip('clear');
clear;

param.L = eye(3);
param.polygen = [-1 -1 1 1;
                 -1 1 1 -1];
param.m = 1.0;
param.mu = 0.3;
param.mu_c = 0.3;
param.g = 9.8;
param.L = diag([param.mu * param.m * param.g, param.mu * param.m * param.g, 3.06 * param.mu * param.m * param.g / 4]);
% integral_(-1)^1 integral_(-1)^1 sqrt(x^2 + y^2)dx dy = 4/3 (sqrt(2) + sinh^(-1)(1))≈3.06078
param.dt = 0.001;
param.ctrldt = 0.01;
param.predt = 0.2;
param.v_star = 5;

param.ctrl = 1;
param.z_rec = [];
param.u_rec = [];
param.x_rec = [];


param.N = 35;
N = param.N;
    
% Model data
A = sdpvar(repmat(5,1,N), repmat(5,1,N), 'full');
B = sdpvar(repmat(5,1,N), repmat(3,1,N), 'full');
nx = 5; % Number of states
nu = 3; % Number of inputs

% MPC data
Q = 20 * diag([1, 1, 2, 0.1, 0.1]);
R = 1 * diag([1, 1, 1]);
W = 2 * diag([0.1, 1, 1]);

dt = param.dt;

u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
u_star = sdpvar(repmat(nu,1,N), repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x_star = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
z = binvar(repmat(3,1,N), repmat(1,1,N));

constraints = [];
objective   = 0;
for k = 1:N
    objective = objective +(x{k}-x_star{k})'*Q*(x{k}-x_star{k}) + (u{k}-u_star{k})'*R*(u{k}-u_star{k}) + z{k}' * W * z{k};

%     Mode1 = [u{k}(3)-u_star{k}(3) == 0, norm(u{k}(2)-u_star{k}(2), 1) <= param.mu_c * (u{k}(1)-u_star{k}(1))];
%     Mode2 = [u{k}(3)-u_star{k}(3) >= 0, u{k}(2)-u_star{k}(2) == param.mu_c * (u{k}(1)-u_star{k}(1))];
%     Mode3 = [u{k}(3)-u_star{k}(3) <= 0, u{k}(2)-u_star{k}(2) == -param.mu_c * (u{k}(1)-u_star{k}(1))];
    Mode1 = [u{k}(3) == 0, norm(u{k}(2), 1) <= param.mu_c * u{k}(1)];
    Mode2 = [u{k}(3) >= 0, u{k}(2) == param.mu_c * u{k}(1)];
    Mode3 = [u{k}(3) <= 0, u{k}(2) == -param.mu_c * u{k}(1)];
    
    constraints = [constraints, implies(z{k}(1), Mode1), implies(z{k}(2), Mode2), implies(z{k}(3), Mode3)];
    constraints = [constraints, sum(z{k}) == 1];
    constraints = [constraints, x{k+1}- x_star{k+1} == A{k}*(x{k}-x_star{k}) + B{k}*(u{k}-u_star{k})] ;
    constraints = [constraints, [0, -50, -50]' <= u{k}<= 50, -50<= x{k}<= 50];
end
objective = objective + x{k+1}'*Q*x{k+1};
constraints = [constraints, -50 <= x{k+1} <= 50];


parameters_in = {x{1}, [A{:}], [B{:}], [u_star{:}], [x_star{:}]};
solutions_out = {[u{:}], [x{:}], [z{:}]};
ops = sdpsettings('verbose',2, 'solver','gurobi');
controller = optimizer(constraints, objective,ops,parameters_in,solutions_out);
param.controller = controller;


% x0, y0, theta0, contact_x, contact_y
x0 = [0, 0, pi/6, -1, 0.5]';
% x0 = [0, 0, 0, -1, 0]';
[x_sim, param] = RungeKutta4th(@pushDynamicsEqn, x0, param, dt, 0.8);

plot_state(x_sim, param);    

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
    xref(:, end+1) = nominalTraj((i-1)*param.dt, param, param.v_star);
end
plot(xref(1:3, :)')

