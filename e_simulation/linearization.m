function [A, B] = linearization(t, param, dt, x)   

[x_star, u_star] = param.traj(t, param);   

x_star3 = x_star(3);   
x_star4 = x_star(4);   
x_star5 = x_star(5);   
u_star1 = u_star(1);   
u_star2 = u_star(2);   
A = ...   
[[0, 0, 0.1325*u_star2*cos(x_star3) - 0.1325*u_star1*sin(x_star3), 0, 0]
[0, 0, 0.1325*u_star1*cos(x_star3) + 0.1325*u_star2*sin(x_star3), 0, 0]
[0, 0, 0, -36.24*u_star2, -36.24*u_star1]
[0, 0, 0, 0, 0]
[0, 0, 0, 0, 0]];
A = dt * A + eye(5);  

B = dt * ...  
[[0.1325*cos(x_star3), 0.1325*sin(x_star3), 0]
[0.1325*sin(x_star3), -0.1325*cos(x_star3), 0]
[-36.24*conj(x_star5), -36.24*conj(x_star4), 0]
[0, 0, 0]
[0, 0, -1.0]];
end  

