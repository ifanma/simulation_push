function [A, B] = linearization(t, param, dt)
    % x_next = A * x + B * u;
    
    if ~exist('v_star', 'var')
        v_star = 10;
    else
        v_star = param.v_star;
    end

    [x_star, u_star] = nominalTraj(t, param, v_star);

    % 下面的A，B形式是固定的
    
    A = ... 
   [[0, 0, (147*u_star(2)*cos(x_star(3)))/50 - (147*u_star(1)*sin(x_star(3)))/50,                 0,                 0];
    [0, 0, (147*u_star(1)*cos(x_star(3)))/50 + (147*u_star(2)*sin(x_star(3)))/50,                 0,                 0];
    [0, 0, 0, -(22491*u_star(2))/10000, -(22491*u_star(1))/10000];
    [0, 0, 0,                 0,                 0];
    [0, 0, 0,                 0,                 0]];

    A = dt * A + eye(5);

    B = ...
    dt * ...
    [[       (147*cos(x_star(3)))/50,        (147*sin(x_star(3)))/50,  0];
    [       (147*sin(x_star(3)))/50,       -(147*cos(x_star(3)))/50,  0];
    [-(22491*x_star(5))/10000, -(22491*x_star(4))/10000,  0];
    [                      0,                       0,  0];
    [                      0,                       0, -1]];

end

