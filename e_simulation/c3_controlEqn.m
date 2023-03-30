function [u, param] = c3_controlEqn(t, x, param)

    controller = param.controller;
    x = double(x);
    
    i = 1;
    for t_ = 0: param.predt: 0 + param.N * param.predt
        [A{i}, B{i}] = c3_linearization(t_, param, param.predt, param.x_ref);
        [x_star_{i}, u_star_{i}] = param.traj(t_, param.x_ref, param);
%         fprintf(param.fid, '%f ', x_star_{i}(1));
        i = i+1;
    end
%         fprintf(param.fid, '\n%f ', x(1));
    
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
    param.ref = x_star_{1}(1:3)';

    
end