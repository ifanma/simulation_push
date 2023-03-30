function cb_timer(obj, event)

    global t_ x0_ u_ param;

    if u_.start_calc
        disp(['control time = ', num2str(t_)]);
        [~, param_] = controlEqn(t_, x0_, param);
        u_.data = param_.u_rec_oneshoot;
        u_.z = param_.z_rec_oneshoot;
        u_.flag = 1;
        u_.diag = param_.diagnostics;
    end
end