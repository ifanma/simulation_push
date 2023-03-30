function [x_star, u_star] = nominalTraj6(t, param)
    
    % 当方块沿着y轴正方向横向移动时的最优控制以及最优轨迹
    umg = param.mu * param.m * param.g;
    v_star = param.v_star;
    x_star =  [0 v_star*t 0 0 -1]';
    u_star = [v_star/(umg), 0, 0]';

end

