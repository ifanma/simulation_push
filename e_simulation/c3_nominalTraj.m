function [x_star, u_star] = c3_nominalTraj(t, x, param)
    
    % 当方块沿着x轴正方向横向移动时的最优控制以及最优轨迹
    % v_star = 10;  % 方块期望速度，随便定一个数字
    % L= 2;         % 方块的边长，接触点需要偏移边长的一半
    % x_star(t) =  [v_star*t 0 0 v_star*t-L/2 0]';
    % u_star(t) = [v_star/(umg), 0, 0]';

    cmd = param.cmd;
    
    R = [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))];

    
    x_cmd(1:2) = x(1:2) + R*cmd(1:2)*t;
    if t> 3.0
        t = 3.0;
    end
    x_cmd(3) = x(3) + 0.8 *atan2(cmd(2), cmd(1)) * t;

    x_star =  [x_cmd -param.l 0]';
    u_star = [cmd(1)/param.L(1,1), 0, 0]';


end

