clear
file = fopen(['mytext_' num2str(1) '.txt'], 'w+');
fprintf(file, ' ');
fclose(file);
file = fopen(['mytext_' num2str(2) '.txt'], 'w+');
fprintf(file, ' ');
fclose(file);

%%
% 
% job = createCommunicatingJob(parcluster('local'),'Type','spmd');%
% job.Type
% job.NumWorkersRange = 2;
% createTask(job,@mytxt,1,{});
% get(job,'Tasks')
% submit(job);
% get(job,'Tasks')
% tic
% wait(job)
% toc

% return;


%%

aaa = 1;
save('data.mat', "aaa")
job = createJob(parcluster());%
job.Type
createTask(job, @mytimer1, 1,{});
createTask(job, @mytimer2, 1,{});
get(job,'Tasks')
submit(job);
get(job,'Tasks')
tic
wait(job)
toc

return 

%% 
t = timer('Name', 't1', 'TasksToExecute', inf,'BusyMode','drop','ExecutionMode',...
    'fixedRate', 'Period',1);
t.TimerFcn = { @callback, 1};
t.ErrorFcn = @(x,y)disp('Error');
t.StartFcn = @(x,y)disp('Start');
t.StopFcn = @(x,y)disp('Stop');
% 
t2 = timer('Name', 't2', 'TasksToExecute', inf,'BusyMode','drop','ExecutionMode',...
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

% 
% t2 = timer('BusyMode','queue','ExecutionMode', 'fixedRate', 'Period',1);
% t2.TimerFcn = { @callback, 2};
% t2.start;