function u = controlEqn(t, x, param)

    uf = [1 0.5]';
    uphi = [0.1]';
    u = [uf; uphi];
end