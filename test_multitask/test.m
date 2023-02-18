%% 1
% tic
% for i=1:3
%     mytxt(i)
% end
% toc

%% 2
% c = parcluster();
% j = createJob(c);
% createTask(j,@mytxt,1,{{1},{2},{3}});
% get(j,'Tasks')
% 
% submit(j);
% tic
% wait(j);
% % taskoutput = fetchOutputs(j);
% % disp(taskoutput{1});
% toc
% 

%% 3
tic
parfor i=1:3
    mytxt(i)
end
toc

