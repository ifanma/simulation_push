function state = mytxt()

    
    t = timer('Name', 't1', 'TasksToExecute', 5,'BusyMode','drop','ExecutionMode',...
        'fixedRate', 'Period',1);
    t.TimerFcn = { @callback, 1};
    t.ErrorFcn = @(x,y)disp('Error');
    t.StartFcn = @(x,y)disp('Start');
    t.StopFcn = @(x,y)disp('Stop');
    % 
    t2 = timer('Name', 't2', 'TasksToExecute', 5,'BusyMode','drop','ExecutionMode',...
        'fixedRate', 'Period',1);
    t2.TimerFcn = { @callback, 2};
    t2.ErrorFcn = @(x,y)disp('Error');
    t2.StartFcn = @(x,y)disp('Start');
    t2.StopFcn = @(x,y)disp('Stop');
    
    t.start();
    t2.start();
    
    pause(3);
    t.stop();
    t2.stop();
    
    timerfind
    delete(t)
    delete(t2)
    delete(timerfind)

    state = 1; %函数状态
end