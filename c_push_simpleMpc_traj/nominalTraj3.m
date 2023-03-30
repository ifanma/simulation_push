function [x_star, u_star] = nominalTraj2(t, param)
    
    % 当方块沿着x轴正方向横向移动时的最优控制以及最优轨迹
    % v_star = 10;  % 方块期望速度，随便定一个数字
    % L= 2;         % 方块的边长，接触点需要偏移边长的一半
    % x_star(t) =  [v_star*t 0 0 v_star*t-L/2 0]';
    % u_star(t) = [v_star/(umg), 0, 0]';
    umg = param.mu * param.m * param.g;
    v_star = param.v_star;

    x_star =  [v_star*t, 1*sin(pi*t), cos(pi*t), -1, 0]';
    u_star = [v_star/(umg), 0, 0]';
    

end

