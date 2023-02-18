function v = calc_v(x, u, param, edge_preset)
    
    % 得到接触点数量
    assert(mod((size(x,1) - 3),2) == 0);
    numc = (size(x,1) - 3)/2;
    assert(size(u,1) == 3 * numc);

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

        T = [cos(x(3)) -sin(x(3)) ; sin(x(3)) cos(x(3))];
%         disp('1')
%         B*u(1:2, :)
%         L*B*u(1:2, :)
%         Jc{i}*L*B*u(1:2, :)
%         T * (Jc{i}*L*B*u(1:2, :) + p2 * u(3, :))
        v{i} = T * (Jc{i}*L*B*u(1:2, :) + p2 * u(3, :));

    end


end

