function state = mytimer1()
 

    file = fopen(['mytext_' num2str(1) '.txt'], 'a+');
%     t = timer('Name', 't1', 'TasksToExecute', 5,'BusyMode','drop','ExecutionMode',...
%         'fixedRate', 'Period',1);
%     t.TimerFcn = { @callback, 2};
%     t.ErrorFcn = @(x,y)disp('Error');
%     t.StartFcn = @(x,y)disp('Start');
%     t.StopFcn = @(x,y)disp('Stop');
    % 
    fprintf(file,'1 %s .. \r\n', datestr(now,'MM.SS.FFF')); 
%     t.start();

    try
    
        tic
        fprintf(file,'2 %s .. \r\n', datestr(now,'MM.SS.FFF'));
        pause(3);
        ct = toc;
    catch ME
        rethrow(ME)
    end


    fprintf(file,'3 %s .. %f\r\n', datestr(now,'MM.SS.FFF'), ct);
%     t.stop();
    
    fprintf(file,'4 %s .. \r\n', datestr(now,'MM.SS.FFF'));
%     delete(t)
%     delete(timerfind)

    fclose(file);

    state = 1; %函数状态
end