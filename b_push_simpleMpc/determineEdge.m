function edge = determineEdge(x, param)
    polygen = [param.polygen(1,:), param.polygen(1,1); param.polygen(2,:), param.polygen(2, 1)];
    assert(mod((size(x,1) - 3),2) == 0);
    numc = (size(x,1) - 3)/2;
    edge = zeros(1, numc);
    for i = 1: numc
        pc{i} = x(2 + 2 * i : 3 + 2 * i, 1); 
        Jc{i} = [1 0 -pc{i}(2); 0 1 pc{i}(1)];
        for j = 1:size(polygen, 2) - 1
            p1 = polygen(:, j) - pc{i}; p1 = p1 ./ norm(p1);
            p2 = polygen(:, j) - polygen(:, j +1); p2 = p2 ./ norm(p2);
            if p1' * p2 == 1
                tc{i} = p2;
                nc{i} = [-p2(2), p2(1)]';
                edge(i) = j;
                break;
            end
        end
    end
end