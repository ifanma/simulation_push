function [u, param] = controlEqn(t, x, param)

    controller = param.controller;

    i = 1;
    for t_ = t: param.predt: t + param.N * param.predt
        [x_star_{i}, u_star_{i}] = param.traj(t_, param);
        
        [A{i}, B{i}] = linearization(t_, param, param.predt, 1);
        S1{i} = [A{i}, B{i}];
        [A{i}, B{i}] = linearization(t_, param, param.predt, 2);
        S2{i} = [A{i}, B{i}];
        [A{i}, B{i}] = linearization(t_, param, param.predt, 3);
        S3{i} = [A{i}, B{i}];
        [A{i}, B{i}] = linearization(t_, param, param.predt, 4);
        S4{i} = [A{i}, B{i}];
        i = i+1;
    end
    inputs = {x, [S1{1:end-1}], [S2{1:end-1}], [S3{1:end-1}], [S4{1:end-1}], [u_star_{1:end-1}], [x_star_{:}]};
    [solutions, diagnostics] = controller{inputs};   
    
    param.u_rec_oneshoot = solutions{1};
    param.x_rec_oneshoot = solutions{2};
    param.z_rec_oneshoot = solutions{3};
    param.e_rec_oneshoot = solutions{4};

    param.u = param.u_rec_oneshoot(:, 1);
    param.x = param.x_rec_oneshoot(:, 1);
    param.z = param.z_rec_oneshoot(:, 1);
    param.e = param.e_rec_oneshoot(:, 1);

%     param.u = [5 -3 0.5]';   % [uf; uphi]
    u = param.u;
end