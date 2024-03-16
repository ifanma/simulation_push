function plot_state(x, param)
    
    figure(1)
    clf;
    hold on

    for k = 1:10:size(x, 2)
        theta = x(3, k);
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        point_box = x(1:2, k) + R * param.polygen;
        point_slider = x(1:2, k) + R * x(4:5, k);
        plot1 = plot([point_box(1,:), point_box(1,1)], [point_box(2,:), point_box(2, 1)], 'b');
        plot1.Color(4) = 0.5;
        plot(point_slider(1), point_slider(2), 'r.');
    end

    axis([-5, 5, -5, 5])
    axis equal
    
end