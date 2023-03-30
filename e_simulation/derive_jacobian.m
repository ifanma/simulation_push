function derive_jacobian(param)

    x_star = sym('x_star', [5 1]);
    u_star = sym('u_star', [3 1]);
    
    f1 = pushDynamicsEqn(0, x_star, param, 1, u_star);
    f2 = pushDynamicsEqn(0, x_star, param, 2, u_star);
    
    digits(4)
    dfdx = vpa(jacobian(f1, x_star), 4);
    dfdu = vpa(jacobian(f1, u_star), 4);
    
    fid=fopen(   'linearization.m','w');
    fprintf(fid, 'function [A, B] = linearization(t, param, dt, x)   \n\n');
    fprintf(fid, '[x_star, u_star] = param.traj(t, param);   \n\n');
    fprintf(fid, 'x_star3 = x_star(3);   \n');
    fprintf(fid, 'x_star4 = x_star(4);   \n');
    fprintf(fid, 'x_star5 = x_star(5);   \n');
    fprintf(fid, 'u_star1 = u_star(1);   \n');
    fprintf(fid, 'u_star2 = u_star(2);   \n');
    fprintf(fid, 'A = ...   \n');
    fprintf(fid, ['[', char(dfdx(1,:)), '\n']);
    fprintf(fid, [char(dfdx(2,:)), '\n']);
    fprintf(fid, [char(dfdx(3,:)), '\n']);
    fprintf(fid, [char(dfdx(4,:)), '\n']);
    fprintf(fid, [char(dfdx(5,:)), '];\n']);
    fprintf(fid, 'A = dt * A + eye(5);  \n\n');
    fprintf(fid, 'B = dt * ...  \n');
    fprintf(fid, ['[', char(dfdu(1,:)), '\n']);
    fprintf(fid, [char(dfdu(2,:)), '\n']);
    fprintf(fid, [char(dfdu(3,:)), '\n']);
    fprintf(fid, [char(dfdu(4,:)), '\n']);
    fprintf(fid, [char(dfdu(5,:)), '];\n']);
    fprintf(fid, 'end  \n\n');


end
