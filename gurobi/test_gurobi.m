
names={'x','y','z'};
model.varnames=names;%定义变量名
model.Q=sparse([1 0.5 0; 0.5 1 0.5; 0 0.5 1]);%目标函数中多次项系数矩阵
model.A=sparse([1 2 3; 1 1 0]);%约束条件系数矩阵
model.obj=[2 0 0];%目标函数中线性系数矩阵
model.rhs=[4 1];
model.sense='>';
gurobi_write(model,'qp.lp');
 
model.vtype='C';
results=gurobi(model);
for v=1:length(names)
    fprintf('%s %e\n',names{v}, results.x(v));
end
fprintf('Obj:%e\n', results.objval);

pause(10)

model.vtype='BCC';
results=gurobi(model);
for v=1:length(names)
    fprintf('%s %e\n',names{v}, results.x(v));
end
fprintf('Obj:%e\n', results.objval);