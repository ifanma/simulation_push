function [x_star, u_star] = nominalTraj5(t, param)
    
    % 沿着正方形走的错误参考轨迹
    umg = param.mu * param.m * param.g;
    v_star = param.v_star;

    t1 = 1.5;
    if t < t1
        x_star =  [v_star*t, 0, 0, 0, 0]';
        u_star = [v_star/(umg), 0, 0]';
    elseif t < 2*t1
        x_star =  [v_star*t1, -v_star*(t-t1), 0, 0, 0]';
        u_star = [v_star/(umg), 0, 0]';
%     elseif t < 3*t1
    else
        x_star =  [v_star*t1-v_star*(t-2*t1), -v_star*t1, 0, 0, 0]';
        u_star = [v_star/(umg), 0, 0]';
    end

end

