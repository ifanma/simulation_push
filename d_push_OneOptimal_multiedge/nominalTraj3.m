function [x_star, u_star] = nominalTraj3(t, param)
    
    % 当方块沿着x轴正方向横向正弦移动时的最优控制以及最优轨迹
    umg = param.mu * param.m * param.g;
    v_star = param.v_star;

    x_star =  [v_star*t, 1*sin(pi*t), cos(pi*t), -1, 0]';
    u_star = [v_star/(umg), 0, 0]';
    

end

