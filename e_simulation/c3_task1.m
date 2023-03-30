function state = c3_task1(x0_, param)

    fileStateID = fopen('c3_state.dat','w');
    state = [0; double(x0_)];
    fwrite(fileStateID, state, 'double');
    fclose(fileStateID);
    M = size(param.uu, 1);
    N = param.N;
    mmap_control = memmapfile('c3_control.dat', 'Writable', true, ...
                    'Format', {'uint8',[1 1],'flag'; 'uint8',[1 1],'diag';...
                               'double',1,'dt'; 'double',[M,N],'u'; ...
                               'double',[1,N],'z'; 'double',[4,N+1],'x'; 'double',[1,3],'ref'});
    mmap_state = memmapfile('c3_state.dat','Format','double', 'Writable', true);
    mmap = memmapfile('c3_joy.dat','Format','double');
    mmap_ref = memmapfile('c3_ref.dat','Format','double');
    logFileID = fopen('c3_Log_task1.txt', 'w+');
    fprintf(logFileID, 'Program started\r\n');

    %% ======================= Vrep初始化 ======================= %%
    sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1',19999,true,false,5000,5);
    offset_xy = [0.075, 0, 0, 0]';      % command_for_joint = p_on_b + offset_xy

    try         % 开始连接Vrep
        if (clientID <= -1)
            fprintf(logFileID, 'Failed connecting to remote API server\r\n');
            assert(0, 'no connection')
        end
        fprintf(logFileID, 'Connected to remote API server\r\n');    
    
        % 以阻塞模式接收数据（service call）
        [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
        if (res==sim.simx_return_ok)
            fprintf(logFileID, 'Number of objects in the scene: %d\r\n',length(objs));
        else
            fprintf(logFileID, 'Remote API function call returned with error code: %d\r\n',res);
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
        [~] = sim.simxGetObjectPosition(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
        [~] = sim.simxGetObjectOrientation(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
        [~] = sim.simxGetObjectPosition(clientID, ps_h, cb_h, sim.simx_opmode_continuous);
        [~] = sim.simxReadForceSensor(clientID, fs_h, sim.simx_opmode_continuous);
        [~] = sim.simxGetJointForce(clientID, Jx_h, sim.simx_opmode_continuous);
        [~] = sim.simxGetJointForce(clientID, Jy_h, sim.simx_opmode_continuous);
            
        % 开始循环 
        x_plot = []; u_plot = []; usize_plot = []; t_plot = []; xpre_plot = {};
        u_rec = zeros(4, param.N); z_plot = []; joint_plot = []; uft_plot = [];
        jft_plot = []; u_index = 0; dt_used = [];
        pos_cmd = pc(1:2);
        pos_joint = pos_cmd;
        angle_round = 0; ang_last = 0;
        fail_loop = 0;
        show = 0;
    
        tic;
        startTime=toc;
        currentTime=toc;
        lastPlotTime = currentTime;
        lastCtrlTime = currentTime;
        lastTime = currentTime;
        while (currentTime-startTime < param.tf)
            currentTime = toc;
%             fprintf(logFileID, 'currentTime = %f, dt = %f\r\n', currentTime, currentTime - lastTime);
            lastTime = currentTime;
    
            joy = mmap.Data;
            param.joy = joy;
%             fprintf(logFileID, 'joy %f, %f, %f, %f, %f, %f\r\n', joy(1), joy(2), joy(3), joy(4), joy(5), joy(6));
            pos_cmd = pos_cmd - [-joy(1), joy(3)]' * 0.000001;
            if joy(2) <-1500
                pos_cmd = [0, 0]';
            end
            pos_cmd(pos_cmd>2) = 2;
            pos_cmd(pos_cmd<-2) = -2;
%             fprintf(file, 'pos_cmd %f, %f\n', pos_cmd(1), pos_cmd(2));
    
            % 获取状态
            [~, pos] = sim.simxGetObjectPosition(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
            [~, ang] = sim.simxGetObjectOrientation(clientID, cb_h, sim.sim_handle_parent, sim.simx_opmode_continuous);
            [~, pos2] = sim.simxGetObjectPosition(clientID, ps_h, cb_h, sim.simx_opmode_continuous);
            [~, ~, force_m, ~] = sim.simxReadForceSensor(clientID, fs_h, sim.simx_opmode_continuous);
            [~, jx_force] = sim.simxGetJointForce(clientID, Jx_h, sim.simx_opmode_continuous);
            [~, jy_force] = sim.simxGetJointForce(clientID, Jy_h, sim.simx_opmode_continuous);
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
            state = [t_; double(x0_)];
            mmap_state.Data = state;
            control_data = mmap_control.Data;
%             fprintf(logFileID, '%f Here here.\r\n', control_data.flag);
            
            % 计算控制
            fprintf(logFileID, '%f, %d\r\n', t_, control_data.flag);
            if control_data.flag == 1
                mmap_control.Data.flag = uint8(0);
                if control_data.diag == 3
                    fprintf(logFileID, 'Solver timeout.\r\n');
                elseif control_data.diag ~= 0
                    fprintf(logFileID, 'Solver failed. please check.\r\n');
                    assert(0, 'solver failed.');
                    break;
                end

                fprintf(logFileID, 'Here here. %f, %f, %f\r\n', t_, size(control_data.x, 2), size(u_rec, 2));
                u_rec = [];
                u_rec(1, :) = control_data.x(1, 1:end-1);
                u_rec(2:4, :) = control_data.u;
                u_rec(:, 1) = [];
                dt_used(:, end+1) = control_data.dt;
                z_plot(:, end + 1) = control_data.z';
                xpre_plot{end + 1} = control_data.x;
                
                if show == 0
                    show = 1;
                    sim.simxAddStatusbarMessage(clientID,['command received! ' char(datetime)],sim.simx_opmode_oneshot);
                end
            end
    
if param.DEBUG ==1
            u_rec(:, 1) = [0.1 / param.L(1,1), 0.02 / param.L(1,1), 0]';
end

            u_ind = find((currentTime - u_rec(1, :)>0.0001) & (currentTime-u_rec(1, :)<=param.loopdt), 1);
            if ~isempty(u_ind)
                u_this = u_rec(2:4, u_ind);
                u_index = u_ind;
            end

            if u_index >= param.N
                break;
            end

%             if isempty(u_rec)
%                 u_rec(:, 1) = [0 0 0 0 ]';
%             end
%             if currentTime - lastCtrlTime > param.predt
%                 u_rec(:, 1) = [];
%                 lastCtrlTime = currentTime;
%             end
%             u_this = u_rec(2:4, 1);

            v = calc_v(x0_, u_this, param, 1);
            u_plot_ = u_this;
            
            % mpc control
            pos_joint = pos_joint + diag([1, 1])*v{1} * param.loopdt;
            pos_joint(pos_joint > 2) = 2;
            pos_joint(pos_joint < -2) = -2;
    
    %         % simple teleoperation
%             pos_joint = pos_cmd;
    
            % 发送控制 
            sim.simxSetJointTargetPosition(clientID, Jy_h, pos_joint(2), sim.simx_opmode_continuous);
            sim.simxSetJointTargetPosition(clientID, Jx_h, pos_joint(1), sim.simx_opmode_continuous);
            [rtn, realJy] = sim.simxGetJointPosition(clientID, Jy_h, 0);
            if rtn ~= 0 ,realJy = 0; end
            [rtn, realJx] = sim.simxGetJointPosition(clientID, Jx_h, 0);
            if rtn ~= 0 ,realJx = 0; end
    
            if currentTime - lastPlotTime > param.plotdt
                ref = mmap_ref.Data;
                x_plot(:, end +1) = [[ref' 0 0], x0_']';
                u_plot(:, end + 1) = u_plot_;
                usize_plot(:, end+1) = u_index;
%                 usize_plot(:, end+1) = size(u_rec, 2);
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
                    fprintf(logFileID, 'fail too many times! check your computer or reduce performance.\r\n');
                    break;
                end
            end
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

    catch ME
        fprintf(logFileID, '%s.\r\n', ME.message);
    end

    save('c3_figureData.mat');
    sim.delete();
    if exist('ME', 'var')
        rethrow(ME);
    end
    state = 1;
end

