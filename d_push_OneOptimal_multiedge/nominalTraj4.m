function [x_star, u_star] = nominalTraj4(t, param)
    
    % 沿着正方形走的正确参考轨迹
    umg = param.mu * param.m * param.g;
    v_star = param.v_star;

    t1 = 1.5;
    if t < t1
        x_star =  [v_star*t, 0, 0, -1, 0]';
        u_star = [v_star/(umg), 0, 0]';
    elseif t < 2*t1
        x_star =  [v_star*t1, -v_star*(t-t1), -pi/2, -1, 0]';
        u_star = [v_star/(umg), 0, 0]';
%     elseif t < 3*t1
    else
        x_star =  [v_star*t1-v_star*(t-2*t1), -v_star*t1, -pi/2, 0, 1]';
        u_star = [v_star/(umg), 0, 0]';
    end

end

