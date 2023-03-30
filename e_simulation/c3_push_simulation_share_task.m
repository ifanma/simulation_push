import mouse3D.*
yalmip('clear');
clear; clc;
disp('Program started');

%% ======================= 数据交换 ======================= %%
fileID = fopen('c3_joy.dat','w');
joy = zeros(6, 1);
fwrite(fileID,joy,'double');
fclose(fileID);
mmap = memmapfile('c3_joy.dat','Format','double', 'Writable', true);
clear('c3_figureData.mat');


%% ======================= spaceNav初始化 ======================= %%
if exist('hDrv', 'var')
    delete(hDrv)
end
hDrv = mouse3Ddrv; %instantiate driver object
addlistener(hDrv,'SenState', @(src, evnt)cb_updateMon(src, evnt, mmap));
% addlistener(hDrv,'ButState',@buttonMon);

%% ======================= 几何参数 ========================== %%
param.l = 0.05;
param.polygen = param.l *[-1 -1 1 1;
                          -1 1 1 -1];
param.m = 0.8;
param.mu = 0.4;         % 地面摩擦
param.mu_c = 0.3;       % 操作摩擦
param.g = 9.81;
umg = param.mu * param.m * param.g;
param.L = diag([0.5, 0.5, 0.2]) * diag([2/umg^2, 2/umg^2,...
                2/(0.03825* umg)^2]);
% 0.03825
c3_derive_jacobian(param);

%% ======================= 疯狂调参 ========================== %%
% MPC objective parameter
param.Q = 10 * diag([1, 1, 0.1, 0, 0]);    % 状态跟踪代价
param.Q_N = 2000 * diag([1, 1, 0.1, 0, 0]);    % 状态跟踪代价
param.R = 0 * diag([0, 1, 0]);       % 控制跟踪代价
param.R_d = 0.1 * diag([1, 1, 0.2]);       % 控制变化代价
param.W = 0.0 * diag([0.8, 1, 1]);        % 状态先验代价，认为不滑动的状态较好
param.V = 0.1 * diag([1, 1, 1]);          % 状态切换代价，认为不切换最好

% MPC constraints parameter
d = 0.01;
param.xl = [-10, -10, -10, -param.l - 0.0005 - d, -param.l *0.9]';
param.xu = [10, 10, 10, -param.l - 0.0005 + d, param.l*0.9]';
param.ul = [0, -0.1, -0.02]';
param.uu = [0.3, 0.1, 0.02]';

% 时间参数
TimerPeriod = 0.05;              % 控制器计算定时器周期
SolverLimitTime = 0.04;          % 优化器时间限制
PredictHorizon = 1.0;           % Mpc预测时域
PredictPeriod = 0.04;           % 优化间隔周期
% N = floor(PredictHorizon/PredictPeriod);      % 优化问题大小
LoopTime = 0.01;                   % 通讯线程单次循环时间
FigureDataRecordPeriod = 0.01;      % 绘画记录数据周期
TotalTime = 30;                  % 程序仿真总时间

param.v_star = param.l ;
% x0 = [0, 0.05, pi/6, -0.05, 0.025]';
% x0 = [0, 0, pi/6, -0.0505, 0.0]';
x0 = [0, 0, 0, -0.0505, 0.0]';
param.DEBUG = 0;

%% ======================= 构建优化问题 ========================== %%
param.predt = PredictPeriod;     
param.loopdt = LoopTime;
param.plotdt = FigureDataRecordPeriod;
param.ctrldt = TimerPeriod;     % 优化计算Timer周期
param.opttimelim = SolverLimitTime;
param.tf = TotalTime;           % 程序循环总时间
param.N = floor(PredictHorizon/param.predt);
param.traj = @c3_nominalTraj;


%% Thread初始化
job = createJob(parcluster());
createTask(job,@c3_task1, 1, {x0, param});
createTask(job,@c3_task2, 1, {param});
job.AutoAttachFiles = false;
submit(job);
get(job,'Tasks')
tic
wait(job);
toc
taskResult = get(job,'Tasks');
taskResult(1)
taskResult(2)

%     f1 = figure(1);
%     set(gcf, 'CurrentCharacter', ' ');

load('c3_figureData.mat');

figure(1)
clf
plot(t_plot, x_plot(6:8, :)', '*');
hold on
plot(t_plot, x_plot(1:3, :)');
% dx = diff(x_plot(1:2, :)')';
% theta = dx(2,:)./(dx(1,:)+0.01);
% plot(t_plot, [theta, 0]);
di = floor(0.2 / param.ctrldt);
if di == 0, di = 1; end
for i = 1:di:size(xpre_plot, 2)
    red_part = xpre_plot{i}(1, :) <= xpre_plot{i}(1, 1) +dt_used(i);
    blue_part = xpre_plot{i}(1, :) >= xpre_plot{i}(1, 1) +dt_used(i) - param.predt;
    plot(xpre_plot{i}(1, red_part), xpre_plot{i}(2, red_part), 'r-');
    plot(xpre_plot{i}(1, red_part), xpre_plot{i}(3, red_part), 'r-');
    plot(xpre_plot{i}(1, red_part), xpre_plot{i}(4, red_part), 'r-');
    plot(xpre_plot{i}(1, blue_part), xpre_plot{i}(2:4, blue_part), 'b-');
end
legend('x', 'y', '\theta', 'x_r', 'y_r', '\theta_r');

figure(2)
clf
subplot(2, 1, 1)
plot(t_plot, u_plot(1:3, :)');
legend('ux', 'uy', '\phi')
subplot(2, 1, 2)
t_diff = diff(t_plot);
plot(t_plot,[0 t_diff]);
hold on
plot(t_plot,FigureDataRecordPeriod* ones(size(t_plot)));

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
plot(t_plot, joint_plot);
legend('cmd_x', 'cmd_y', 'real_x', 'real_y');
subplot(2,1,2)
plot(t_plot, u_plot(1:2, :)');
hold on
plot(t_plot, movavg(uft_plot','linear',10)*diag([0.24/0.75, 1]));
%     plot(t_plot, jft_plot);
legend('cmd_x', 'cmd_y', 'real_x', 'real_y', 'joint_x', 'joint_y');
    

figure(5)
clf
x_plot_diff = diff(x_plot')';
plot(t_plot(1:end-1), movavg(x_plot_diff(1:2, :)','linear', 100));
hold on
plot(t_plot(1:end-1), movavg(x_plot_diff(6:7, :)','linear', 100)');

legend( 'vx', 'vy', 'vx_{ref}', 'vy_{ref}');

%% 析构
delete(timerfind);
delete(hDrv);
disp('Program ended');

%% 结束

function cb_updateMon(src, ~, varargin)
    mmap = varargin{1};
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

    mmap.Data = joy;
    c3_joy2cmd(joy)
end








