function x = RungeKutta4th(equ, x0, param, tstep, tf)
    assert(size(x0, 2) == 1)
    x = zeros([size(x0, 1), tf/tstep]);
    x(:, 1) = x0;

    for i = 1: tf/tstep+1
        t_ = (i - 1)* tstep;
        K1 = tstep * equ(t_          , x(:, i)       , param);
        K2 = tstep * equ(t_ + tstep/2, x(:, i) + K1/2, param);
        K3 = tstep * equ(t_ + tstep/2, x(:, i) + K2/2, param);
        K4 = tstep * equ(t_ + tstep  , x(:, i) + K3  , param);
        x(:, i + 1) = x(:, i) + (K1 + 2 *K2 + 2 *K3 + K4 )/6;
        if (mod(i/tf*tstep,0.1) == 0 )
            disp(['progress:' num2str(i/tf*tstep*100) '%']);
        end
    end
end