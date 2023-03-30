import mouse3D.*


disp('Program started');
%% param
global deadzone_m
deadzone_m = 100;

%% Vrep
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

%% spaceNav
if exist('hDrv', 'var')
    delete(hDrv)
end
hDrv = mouse3Ddrv; %instantiate driver object
addlistener(hDrv,'SenState',@updateMon);
addlistener(hDrv,'ButState',@buttonMon);
global joy

if (clientID>-1)
    disp('Connected to remote API server');
        
    % Now try to retrieve data in a blocking fashion (i.e. a service call):
    [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
    if (res==sim.simx_return_ok)
        fprintf('Number of objects in the scene: %d\n',length(objs));
    else
        fprintf('Remote API function call returned with error code: %d\n',res);
    end

    [~, Jx_h] = sim.simxGetObjectHandle(clientID, 'Jx', sim.simx_opmode_blocking);
    [~, Jy_h] = sim.simxGetObjectHandle(clientID, 'Jy', sim.simx_opmode_blocking);
    [~, cb_h] = sim.simxGetObjectHandle(clientID, './Cuboid[0]', sim.simx_opmode_blocking);

    sim.simxSetJointPosition(clientID, Jx_h, 0, sim.simx_opmode_blocking);
    sim.simxSetJointPosition(clientID, Jy_h, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, Jx_h, 0, sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, Jy_h, 0, sim.simx_opmode_blocking);
    pause(2);
    sim.simxSetObjectPosition(clientID, cb_h, 1, [0, 0, 0], sim.simx_opmode_blocking);
    sim.simxSetObjectQuaternion(clientID, cb_h, 1, [0, 0, 0, 1], sim.simx_opmode_blocking);
    pause(2);

    % Now retrieve streaming data (i.e. in a non-blocking fashion):
    rec = [];
    pos_cmd = [0, 0];
    tic;
    startTime=toc;
    currentTime=toc;
    lastplotTime = toc;
    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming); % Initialize streaming
    while (currentTime-startTime < 50)   
        
        currentTime = toc;
%         fprintf('joy %f, %f\n', joy(1), joy(2))
        pos_cmd = pos_cmd - [joy(1), -joy(3)]' * 0.000001;
        if joy(2) <-1500
            pos_cmd = [0, 0];
        end
        pos_cmd(pos_cmd>2) = 2;
        pos_cmd(pos_cmd<-2) = -2;
        fprintf('pos_cmd %f, %f\n', pos_cmd(1), pos_cmd(2))
    
        sim.simxSetJointTargetPosition(clientID, Jx_h, pos_cmd(1), sim.simx_opmode_continuous);
        sim.simxSetJointTargetPosition(clientID, Jy_h, pos_cmd(2), sim.simx_opmode_continuous);
        [rtn, pos] = sim.simxGetJointPosition(clientID, Jy_h, 0);
        if rtn ~= 0
            pos = 0;
        end
        
        if currentTime - lastplotTime > 0.01
            ref = pos_cmd(2);
            rec(:, end +1) = [ref, pos]';
            lastplotTime = currentTime;
        end

        pause(0.001)
    end
        
    % 
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);

    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID);

    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end


sim.delete(); % call the destructor!
delete(hDrv)
delete(h1);
delete(h2);

disp('Program ended');

plot(rec')
legend('ref', 'rel')


function updateMon(src, varargin)

%     tx = angvec2tr( src.Sen.Rotation.Angle*pi/180, [src.Sen.Rotation.X src.Sen.Rotation.Y src.Sen.Rotation.Z] );
%     tr2eul(tx);

    global joy deadzone_m
    joy = [src.Sen.Translation.X, src.Sen.Translation.Y, src.Sen.Translation.Z...
           src.Sen.Rotation.X, src.Sen.Rotation.Y, src.Sen.Rotation.Z];
    for i = 1:3
        if abs(joy(i)) < deadzone_m
            joy(i) = 0;
        elseif joy(i) > 0
            joy(i) = joy(i) - deadzone_m;
        elseif joy(i) < 0
            joy(i) = joy(i) + deadzone_m;
        end
    end

end

function buttonMon(src,varargin)
    global key1
    if src.Key.IsKeyDown(1)
        key1 = 1
    end
    disp('here');
%     set(obj.bth(3), 'String',  num2str(src.Key.IsKeyDown(1),'%i') );
%     set(obj.bth(4), 'String',  num2str(src.Key.IsKeyDown(2),'%i') );
end
