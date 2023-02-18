function [u, param] = controlEqn(t, x, param)

%     uf = [1 0.5]';
%     uphi = [0.1]';
%     u = [uf; uphi];

    controller = param.controller;

    i = 1;
    for t_ = t: param.predt: t + param.N * param.predt
        [A{i}, B{i}] = linearization(t_, param, param.predt);
        [x_star_{i}, u_star_{i}] = param.traj(t_, param);
        i = i+1;
    end
    inputs = {x, [A{1:end-1}], [B{1:end-1}], [u_star_{1:end-1}], [x_star_{:}]};
    [solutions, diagnostics] = controller{inputs};   
    
    u = solutions{1}(:,1);
    z = solutions{3}(:,1);

%     u = [5 -3 0.5]';   % [uf; uphi]
    
    param.u = u;
    param.z = z;
    
end