function [xd, param] = pushDynamicsEqn(t, x, param, edge_preset, u_preset)

    % 得到接触点数量
    assert(mod((size(x,1) - 3),2) == 0);
    numc = (size(x,1) - 3)/2;

    
%% controller
    % 得到控制,u = [ufn, uft, u_phi]
    if exist("u_preset", "var")
        u = u_preset;
    else
        if mod(t, param.ctrldt) == 0 && nargout ==2
            [u, param] = controlEqn(t, x, param);
        else
            u = param.u;
        end
    end

    % 记录数据
    if nargout == 2
        if isfield(param, 'z_rec')
            [~, ind] = max(param.z_rec_oneshoot);
            ind(ind == 1) = 0;
            ind(ind == 2) = 1;
            ind(ind == 3) = -1;
            param.z_rec(:, end +1) = ind';
        end
        if isfield(param, 'u_rec')
            param.u_rec(:, end +1) = param.u;
        end
        if isfield(param, 'x_rec')
            param.x_rec(:, end +1) = x;
        end
        if isfield(param, 'e_rec')
            [~, ind] = max(param.e_rec_oneshoot);
            param.e_rec(:, end +1) = ind';
        end

        xd = x; 
        % 由于需要根据控制器计算结果改变x的值，在这里临时复用xd来传递新的x
        return
    end

%% dynamics

    % 计算必要参数
    R = [cos(x(3)) -sin(x(3)) 0; sin(x(3)) cos(x(3)) 0; 0 0 1];
    L = param.L;
    C = zeros(2 * numc, numc);
    
    % 加入第一个点构成封闭的多边形
    polygen = [param.polygen(1,:), param.polygen(1,1); param.polygen(2,:), param.polygen(2, 1)];
    
    % 确定当前接触点在哪条边
    if exist("edge_preset", "var")
        edge = edge_preset;
    else
        edge = determineEdge(x, param);
        judge = edge ~= find(param.e == max(param.e));
        if edge == 0 || judge(1)
            pause(1);
        end
    end

    % 遍历接触点，得到系统方程
    for i = 1: numc

        % 得到接触雅可比
        pc{i} = x(2 + 2 * i : 3 + 2 * i, 1); 
        Jc{i} = [1 0 -pc{i}(2); 0 1 pc{i}(1)];
        
        % 获得接触点处法向量和切向量
        p1 = polygen(:, edge(i)) - pc{i}; p1 = p1 ./ norm(p1);
        p2 = polygen(:, edge(i)) - polygen(:, edge(i) +1); p2 = p2 ./ norm(p2);
        tc{i} = p2;
        nc{i} = [-p2(2), p2(1)]';

        % 得到B矩阵
        B(:, i) = Jc{i}' * nc{i};
        B(:, numc + i) = Jc{i}' * tc{i};

        % 得到C矩阵
        C(2*i - 1:2 * i, i) = p2;
    end

    xd = [R * L * B, zeros(3, numc); zeros(2 * numc, 2 * numc), C] * u + 0.1 * rand(5, 1);


end


