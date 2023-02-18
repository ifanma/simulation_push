import mouse3D.*
yalmip('clear');
clear; clc;
disp('Program started');

%% 参数
global joy t_ x0_ u_ z_ param
u_.flag = 0;
u_.start_calc = 0;

%% Vrep初始化
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19999,true,false,5000,5);

%% spaceNav初始化
if exist('hDrv', 'var')
    delete(hDrv)
end
hDrv = mouse3Ddrv; %instantiate driver object
addlistener(hDrv,'SenState',@cb_updateMon);
% addlistener(hDrv,'ButState',@buttonMon);

%% Controller-参数
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
% integral_(-1)^1 integral_(-1)^1 sqrt(x^2 + y^2)dx dy = 4/3 (sqrt(2) + sinh^(-1)(1))≈3.06078

derive_jacobian(param);

%% ======================= 疯狂调参 ========================== %%
% MPC objective parameter
Q = 2 * diag([20, 20, 10, 0, 5]);    % 状态跟踪代价
R = 1 * diag([100, 10, 10]);       % 控制跟踪代价
W = 0.2 * diag([0.5, 1, 1]);        % 状态先验代价，认为不滑动的状态较好
V = 0.2 * diag([1, 1, 1]);          % 状态切换代价，认为不切换最好

% MPC constraints parameter
d = 0.004;
xl = [-10, -10, -10, -param.l - 0.0005 - d, -param.l *0.9]';
xu = [10, 10, 10, -param.l - 0.0005 + d, param.l*0.9]';
ul = [0, -10, -10]';
uu = [1, 10, 10]';

% MPC controller period parameter
TimerPeriod = 0.2;
PredicrPeriod = 0.3;
TotalTime = 5;

param.predt = 0.02;     % 优化间隔周期
param.v_star = param.l * 5;
% x0_ = [0, 0.05, pi/6, -0.05, 0.025]';
x0_ = [0, 0, 0, -0.05, 0]';

DEBUG = 1;
% =================================================================== %

%% Build Controller Model
param.loopdt = 0.002;
param.plotdt = 0.01;    % 绘画周期
param.ctrldt = TimerPeriod;     % 优化计算Timer周期
param.tf = TotalTime;         % 程序循环时间
param.N = floor(PredicrPeriod/param.predt);
N = param.N;
param.traj = @nominalTraj;

% build model variable
A = sdpvar(repmat(5,1,N), repmat(5,1,N), 'full');
B = sdpvar(repmat(5,1,N), repmat(3,1,N), 'full');
nx = 5; % Number of states
nu = 3; % Number of inputs

u = sdpvar(repmat(nu,1,N), repmat(1,1,N));
u_star = sdpvar(repmat(nu,1,N), repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x_star = sdpvar(repmat(nx,1,N+1), repmat(1,1,N+1));
x0 = sdpvar(nx,1);
z = binvar(repmat(3,1,N), repmat(1,1,N));

% build problem
constraints = [];
objective   = 0;
constraints = [constraints, x{1} == x0];
for k = 1:N
    objective = objective +(x{k}-x_star{k})'*Q*(x{k}-x_star{k}) + (u{k}-u_star{k})'*R*(u{k}-u_star{k}) + z{k}' * W * z{k};
    if k ~= 1
         objective = objective + (z{k} - z{k-1})' * V * (z{k} - z{k-1});
    end
    Mode1 = [u{k}(3) == 0, -param.mu_c*u{k}(1) <= u{k}(2)<= param.mu_c*u{k}(1)];
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
ops = sdpsettings('verbose', 1, 'solver','gurobi', 'savedebug', '0', 'gurobi.timelimit', 0.2);
controller = optimizer(constraints, objective,ops,parameters_in,solutions_out);
param.controller = controller;
disp('Controller build finished.')


%% Timer初始化
t = timer('StartDelay', 0, 'Period', param.ctrldt, 'TasksToExecute', 100000, 'ExecutionMode', 'fixedRate');
% t.TimerFcn = { @cb_timer, param};
t.TimerFcn = @c1_cb_timer;
t.StartFcn = @(x,y)disp('Control Thread Started!');
t.StopFcn = @(x,y)disp('Control Thread Stopped!');
f1 = figure(1);
set(gcf, 'CurrentCharacter', ' ');

%% vectors
offset_xy = [0.075, 0, 0, 0]';      % command_for_joint = p_on_b + offset_xy


%% 开始连接Vrep
try
    if (clientID <= -1)
        disp('Failed connecting to remote API server');
        assert(0, 'no connection')
    end
    disp('Connected to remote API server');    

    % 以阻塞模式接收数据（service call）
    [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
    if (res==sim.simx_return_ok)
        fprintf('Number of objects in the scene: %d\n',length(objs));
    else
        fprintf('Remote API function call returned with error code: %d\n',res);
    end

    [~, Jx_h] = sim.simxGetObjectHandle(clientID, 'Jx', sim.simx_opmode_blocking);
    [~, Jy_h] = sim.simxGetObjectHandle(clientID, 'Jy', sim.simx_opmode_blocking);
    [~, cb_h] = sim.simxGetObjectHandle(clientID, './Cuboid[0]', sim.simx_opmode_blocking);
    [~, ps_h] = sim.simxGetObjectHandle(clientID, './pusher', sim.simx_opmode_blocking);
    [~, fs_h] = sim.simxGetObjectHandle(clientID, 'ForceSensor', sim.simx_opmode_blocking);

    sim.simxSetJointTargetPosition(clientID, Jx_h, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, Jy_h, 0, sim.simx_opmode_blocking);
    pause(2);
    quat = angle2quat(x0_(3),0,0);
    sim.simxSetObjectPosition(clientID, cb_h, 1, [x0_(1), x0_(2), 0], sim.simx_opmode_blocking);
    sim.simxSetObjectQuaternion(clientID, cb_h, 1, [quat(2:4), quat(1)], sim.simx_opmode_blocking);

    T = [cos(x0_(3)) -sin(x0_(3)) 0 x0_(1); sin(x0_(3)) cos(x0_(3)) 0 x0_(2); 0 0 1 0; 0 0 0 1];
    pc = T * [x0_(4), x0_(5), 0, 1]' + offset_xy;
    sim.simxSetJointTargetPosition(clientID, Jx_h, pc(1), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, Jy_h, pc(2), sim.simx_opmode_blocking);
    pause(2);

    %% 获取状态
    [rtn, pos] = sim.simxGetObjectPosition(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
    [rtn, ang] = sim.simxGetObjectOrientation(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
    [rtn, pos2] = sim.simxGetObjectPosition(clientID, ps_h, cb_h, sim.simx_opmode_continuous);
    [rtn, ft] = sim.simxReadForceSensor(clientID, fs_h, sim.simx_opmode_continuous);
    [rtn, jx_force] = sim.simxGetJointForce(clientID, Jx_h, sim.simx_opmode_continuous);
    [rtn, jy_force] = sim.simxGetJointForce(clientID, Jy_h, sim.simx_opmode_continuous);
        
    % 开始循环 
    x_plot = []; u_plot = []; usize_plot = []; t_plot = []; 
    u_rec = []; z_plot = []; joint_plot = []; uft_plot = []; jft_plot = []; 
    pos_cmd = pc(1:2);
    pos_joint = pos_cmd;
    angle_round = 0; ang_last = 0;
    fail_loop = 0;
if DEBUG == 0
    start(t);
end
    tic;
    startTime=toc;
    currentTime=toc;
    lastPlotTime = currentTime;
    lastCtrlTime = currentTime;
    lastTime = currentTime;
    while (currentTime-startTime < param.tf)   
        
        currentTime = toc;
%         fprintf('delta_time = %f\n', currentTime - lastTime);
        lastTime = currentTime;

%         fprintf('joy %f, %f\n', joy(1), joy(2))
        pos_cmd = pos_cmd - [-joy(1), joy(3)]' * 0.000001;
        if joy(2) <-1500
            pos_cmd = [0, 0]';
        end
        pos_cmd(pos_cmd>2) = 2;
        pos_cmd(pos_cmd<-2) = -2;
%         fprintf('pos_cmd %f, %f\n', pos_cmd(1), pos_cmd(2));

        % 获取状态
        [rtn, pos] = sim.simxGetObjectPosition(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
        [rtn, ang] = sim.simxGetObjectOrientation(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
        [rtn, pos2] = sim.simxGetObjectPosition(clientID, ps_h, cb_h, sim.simx_opmode_continuous);
        [rtn, state, force_m, torque_m] = sim.simxReadForceSensor(clientID, fs_h, sim.simx_opmode_continuous);
        [rtn, jx_force] = sim.simxGetJointForce(clientID, Jx_h, sim.simx_opmode_continuous);
        [rtn, jy_force] = sim.simxGetJointForce(clientID, Jy_h, sim.simx_opmode_continuous);
        ang = ang(3);
        if ang - ang_last > pi
            angle_round = angle_round - 1;
        elseif ang - ang_last < -pi
            angle_round = angle_round + 1;
        end
        ang_last = ang;
        ang = ang + angle_round * 2 * pi;

        x0_ = [pos(1), pos(2), ang, pos2(1), pos2(2)]';
        t_ = currentTime;
        if abs(x0_(4) + 0.0505) > 0.1
            break;
        end

        % 计算控制
        u_.start_calc = 1;
        if u_.flag == 1
            u_.flag = 0;
            if u_.diag ~= 0
                disp('Solver failed. please check.');
                break;
            end
            u_rec = u_.data;
            [~, ind] = max(u_.z);
            ind(ind == 1) = 0;
            ind(ind == 2) = 1;
            ind(ind == 3) = -1;
            z_plot(:, end + 1) = ind';
        end
        
        if isempty(u_rec)
            u_rec(:, 1) = [0 0 0 ]';
        end
if DEBUG ==1
        u_rec(:, 1) = [0.1/ param.L(1,1), 0, 0]';
end
        v = calc_v(x0_, u_rec(:, 1), param, 1);
        u_plot_ = u_rec(:, 1);
        if currentTime - lastCtrlTime > param.predt
            u_rec(:, 1) = [];
            lastCtrlTime = currentTime;
        end
        
        % mpc control
        pos_joint = pos_joint + v{1} * param.loopdt;
        pos_joint(pos_joint>2) = 2;
        pos_joint(pos_joint<-2) = -2;


%         % simple teleoperation
%         pos_joint = pos_cmd;

        % 发送控制 
        sim.simxSetJointTargetPosition(clientID, Jx_h, pos_joint(1), sim.simx_opmode_continuous);
        sim.simxSetJointTargetPosition(clientID, Jy_h, pos_joint(2), sim.simx_opmode_continuous);
        [rtn, realJy] = sim.simxGetJointPosition(clientID, Jy_h, 0);
        if rtn ~= 0 ,realJy = 0; end
        [rtn, realJx] = sim.simxGetJointPosition(clientID, Jx_h, 0);
        if rtn ~= 0 ,realJx = 0; end
        
        if strcmpi(get(gcf,'CurrentCharacter'),'q')
            break;
        end

        if currentTime - lastPlotTime > param.plotdt
            ref = param.traj(currentTime, param);
            x_plot(:, end +1) = [ref', x0_']';
            u_plot(:, end + 1) = u_plot_;
            usize_plot(:, end+1) = size(u_rec, 2);
            t_plot(:, end+1) = currentTime;
            joint_plot(:, end+1) = [pos_joint; realJx; realJy];
            R = [cos(x0_(3)) -sin(x0_(3)) ; sin(x0_(3)) cos(x0_(3)) ]';
            uft_plot(:, end+1) = diag([-1, 1]) * R * [force_m(1) force_m(2)]';
            jft_plot(:, end+1) = -R * [jx_force, jy_force]';
            lastPlotTime = currentTime;
        end

        periodTime = toc;
        delay_time = param.loopdt - (periodTime - currentTime);
        if delay_time < 0
            fail_loop = fail_loop + 1;
            if fail_loop > 50
                disp('fail too many times! check your computer or reduce performance.')
                break;
            end
        end
%         assert(fail_loop < 50)
        pause(delay_time);
    end
      
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);

    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID);

%     sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
%     sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
%     sim.simxPauseSimulation(clientID, sim.simx_opmode_blocking);
    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(clientID);

    figure(1)
    clf
    plot(t_plot, x_plot(6:8, :)', '*');
    hold on
    plot(t_plot, x_plot(1:3, :)')
    legend('x', 'y', '\theta', 'x_r', 'y_r', '\theta_r')

    figure(2)
    clf
    plot(t_plot, u_plot(1:3, :)');
    legend('ux', 'uy', '\phi')

    figure(3)
    clf
    subplot(2, 1, 1)
    plot(t_plot, usize_plot)

    subplot(2, 1, 2)
    imagesc(flipud(z_plot)); axis tight; grid off; 
    caxis([-1 1]); 
    colormap([0 0 1;0 1 0;1 1 0])
    colorbar
    title('接触状态')

    figure(4)
    clf
    subplot(2,1,1)
%     plot(t_plot, [zeros(4,1) diff(joint_plot')']);
    plot(t_plot, joint_plot);
    legend('cmd_x', 'cmd_y', 'real_x', 'real_y')
    subplot(2,1,2)
    plot(t_plot, u_plot(1:2, :)');
    hold on
    plot(t_plot, uft_plot);
%     plot(t_plot, jft_plot);
    legend('cmd_x', 'cmd_y', 'real_x', 'real_y', 'joint_x', 'joint_y')
    



catch ME
    disp('error occured.');
end

%% 析构
t.stop;
delete(timerfind)
sim.delete();
delete(hDrv)
delete(t);

disp('Program ended');

if exist('ME', 'var')
    rethrow(ME)
end




function cb_updateMon(src, varargin)

%     tx = angvec2tr( src.Sen.Rotation.Angle*pi/180, [src.Sen.Rotation.X src.Sen.Rotation.Y src.Sen.Rotation.Z] );
%     tr2eul(tx);

    global joy 
    deadzone_m = 100;
    joy = [src.Sen.Translation.X, src.Sen.Translation.Y, src.Sen.Translation.Z...
           src.Sen.Rotation.X, src.Sen.Rotation.Y, src.Sen.Rotation.Z];
    for i = 1:3
        if abs(joy(i)) < deadzone_m
            joy(i) = 0;
        elseif joy(i) > 0
            joy(i) = joy(i) - deadzone_m;
        elseif joy(i) < 0
            joy(i) = joy(i) + deadzone_m;
        end
    end

end




