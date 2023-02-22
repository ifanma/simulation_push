function state = c2_task2(param)


    fileControlID = fopen('c2_control.dat','w');
    M = size(param.uu, 1);
    N = param.N;
    fwrite(fileControlID, 1, 'uint8');
    fwrite(fileControlID, 0, 'uint8');
    fwrite(fileControlID, 1, 'double');
    fwrite(fileControlID, zeros(M, N), 'double');
    fwrite(fileControlID, zeros(1, N), 'double');
    fwrite(fileControlID, zeros(4, N+1), 'double');
    fclose(fileControlID);
    mmap_control = memmapfile('c2_control.dat', 'Writable', true, ...
                    'Format', {'uint8',[1 1],'flag'; 'uint8',[1 1],'diag';...
                               'double',1,'dt'; 'double',[M,N],'u'; ...
                               'double',[1,N],'z'; 'double',[4,N+1],'x'});
    mmap_state = memmapfile('c2_state.dat','Format','double');
    logFileID = fopen('c2_Log_task2.txt', 'w+');
    fprintf(logFileID, 'Program started\r\n');

    try         % 开始连接Vrep
        N = param.N;
        
        Q = param.Q; R = param.R; W = param.W; V = param.V;
        Qn = param.Q_N; Rd = param.R_d;
        xl = param.xl; xu = param.xu;
        ul = param.ul; uu = param.uu;

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
                 objective = objective + (u{k} - u{k-1})' * Rd * (u{k} - u{k-1});
            end
            Mode1 = [u{k}(3) == 0, -param.mu_c*u{k}(1) <= u{k}(2)<= param.mu_c*u{k}(1)];
            Mode2 = [u{k}(3) >= 0, u{k}(2) == param.mu_c * u{k}(1)];
            Mode3 = [u{k}(3) <= 0, u{k}(2) == -param.mu_c * u{k}(1)]; 
            constraints = [constraints, implies(z{k}(1), Mode1), implies(z{k}(2), Mode2), implies(z{k}(3), Mode3)];
            constraints = [constraints, sum(z{k}) == 1];
            constraints = [constraints, x{k+1}- x_star{k+1} == A{k}*(x{k}-x_star{k}) + B{k}*(u{k}-u_star{k})] ;
            constraints = [constraints, ul<= u{k}<= uu, xl<= x{k}<= xu];
        end
        objective = objective + (x{k+1}- x_star{k+1})'*Qn*(x{k+1}- x_star{k+1});
        constraints = [constraints, xl<= x{k+1}<= xu];
        
        parameters_in = {x0, [A{:}], [B{:}], [u_star{:}], [x_star{:}]};
        solutions_out = {[u{:}], [x{:}], [z{:}]};
        ops = sdpsettings('verbose', 1, 'solver','gurobi', 'savedebug', false, 'gurobi.timelimit', param.opttimelim);
        controller = optimizer(constraints, objective,ops,parameters_in,solutions_out);
        param.controller = controller;
        fprintf(logFileID, 'Controller build finished.\r\n');
    
        %% Timer初始化
        t = timer('StartDelay', 0, 'Period', param.ctrldt, 'TasksToExecute', inf,...
            'BusyMode', 'queue', 'ExecutionMode', 'fixedRate');
        % t.TimerFcn = { @cb_timer, param};
        t.TimerFcn = {@c2_timer_cb, logFileID, param, mmap_state, mmap_control};
        t.StartFcn = @(x,y)fprintf(logFileID, 'Control Thread Started!\r\n');
        t.StopFcn = @(x,y)fprintf(logFileID,'Control Thread Stopped!\r\n');
        
        t_star = tic;
        while toc(t_star) < 10
            fprintf(logFileID, 'Waiting for starting communication thread.\r\n');
            pause(0.05);
            if mmap_control.Data.flag == 0
                break;
            end
        end
        if param.DEBUG == 0
            t.start;
        end
        t_star = tic;
        while toc(t_star) < param.tf
            pause(0.05);
            fprintf(logFileID, 'wait for task final time: %f.\r\n', toc(t_star));
            if mmap_control.Data.diag ~= 3 && mmap_control.Data.diag ~= 0
                fprintf(logFileID, 'Solver failed\r\n');
                break;
            end
        end
        t.stop;
        fprintf(logFileID, 'Timer stoped.\r\n');

    catch ME
        fprintf(logFileID, 'Error in task: %s.\r\n', ME.message);
    end

    delete(timerfind);
    state = 1;
    if exist('ME', 'var')
        rethrow(ME);
    end
end


function c2_timer_cb(~, ~, logFileID, param, mmap_state, mmap_control)
    try
        state = mmap_state.Data;
        t = state(1);
        x = state(2:6);


        t_start = tic;
        [~, param_] = controlEqn(t, x, param);
        usedTime = toc(t_start);
    
        mmap_control.Data.u = param_.u_rec_oneshoot;
    %     mmap_control.Data.u = repmat([0.25 / param.L(1,1), 0, 0]', 1, param.N);
    %     pause(0.2);
    
        [~, ind] = max(param_.z_rec_oneshoot);
        ind(ind == 1) = 0;
        ind(ind == 2) = 1;
        ind(ind == 3) = -1;
        mmap_control.Data.z = ind';
        mmap_control.Data.x(1, :) = t:param_.predt:t + param_.N*param_.predt;
        mmap_control.Data.x(2:4, :) = param_.x_rec_oneshoot(1:3, :);
        mmap_control.Data.diag = uint8(param_.diagnostics);
        mmap_control.Data.flag = uint8(1);
        mmap_control.Data.dt = usedTime;

        fprintf(logFileID,'Timer in. t = %f, x1 = %f, %d, %d\r\n', state(1), state(2), size(param_.u_rec_oneshoot, 2), size(param_.x_rec_oneshoot, 2));

    catch ME
        fprintf(logFileID,'Error in timer: %s\r\n', ME.identifier);
    end

end

