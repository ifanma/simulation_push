function [u, param] = controlEqn(t, x, param)

    controller = param.controller;
    x = double(x);
    
    i = 1;
    for t_ = t: param.predt: t + param.N * param.predt
        [A{i}, B{i}] = linearization(t_, param, param.predt, x);
        [x_star_{i}, u_star_{i}] = param.traj(t_, param);
        i = i+1;
    end
    inputs = {x, [A{1:end-1}], [B{1:end-1}], [u_star_{1:end-1}], [x_star_{:}]};
    [solutions, diagnostics] = controller{inputs};   
    
    param.diagnostics = diagnostics;
    param.u_rec_oneshoot = solutions{1};
    param.x_rec_oneshoot = solutions{2};
    param.z_rec_oneshoot = solutions{3};

    u = param.u_rec_oneshoot(:, 1);
%     u = [1 -0.3 0]';   % [uf; uphi]
    param.u = u;
    param.z = param.z_rec_oneshoot(:, 1);

    
end