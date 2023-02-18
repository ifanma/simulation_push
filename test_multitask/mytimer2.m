function state = mytimer2()
 
   file = fopen(['mytext_' num2str(2) '.txt'], 'a+');
    t = timer('Name', 't2', 'TasksToExecute', 5,'BusyMode','drop','ExecutionMode',...
        'fixedRate', 'Period',1);
    t.TimerFcn = { @callback, 2};
    t.ErrorFcn = @(x,y)disp('Error');
    t.StartFcn = @(x,y)disp('Start');
    t.StopFcn = @(x,y)disp('Stop');
    % 
    fprintf(file,'1 %s .. \r\n', datestr(now,'MM.SS.FFF')); 
    t.start();
    
    tic
    fprintf(file,'2 %s .. \r\n', datestr(now,'MM.SS.FFF'));
    pause(3);
    ct = toc;


    fprintf(file,'3 %s .. %f\r\n', datestr(now,'MM.SS.FFF'), ct);
    t.stop();
    
    fprintf(file,'4 %s .. \r\n', datestr(now,'MM.SS.FFF'));
    delete(t)
    delete(timerfind)

    fclose(file);

    state = 1; %函数状态
end