function [x, param] = RungeKutta4th(equ, x0, param, tstep, tf)
    assert(size(x0, 2) == 1)
    x = zeros([size(x0, 1), tf/tstep]);
    x(:, 1) = x0;

    for i = 1: tf/tstep+1
        t_ = (i - 1)* tstep;
        [x(:, i), param]  = equ(t_          , x(:, i)       , param);   
        % 由于需要根据控制器计算结果改变x的值，在这里临时复用xd来传递新的x
        K1     = equ(t_          , x(:, i)       , param);
        K2     = equ(t_ + tstep/2, x(:, i) + K1/2* tstep, param);
        K3     = equ(t_ + tstep/2, x(:, i) + K2/2* tstep, param);
        K4     = equ(t_ + tstep  , x(:, i) + K3* tstep  , param);
        x(:, i + 1) = x(:, i) + (K1 + 2 * K2 + 2 * K3 + K4 )/6 * tstep;
        if (mod(i/tf*tstep,0.1) == 0 )
            disp(['progress:' num2str(i/tf*tstep*100) '%']);
        end
    end
end