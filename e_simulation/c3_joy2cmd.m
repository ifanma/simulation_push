function cmd = c3_joy2cmd(joy)
    cmd = diag([0.0001, 0.01, 0.005]) * [-joy(2), joy(5), 0]';
    uplim = [0.03, 0.01, 0.005]';
    lowlim = [0, -0.01, -0.005]';
    cmd(cmd > uplim) = uplim(cmd > uplim);
    cmd(cmd < lowlim) = lowlim(cmd < lowlim);
end