function [AccelX, AccelY, next_update_time_out] = AdmitCtlrPruning(t, next_update_time_in, c, wrench, X)
    global Ax
    global Ay
%     disp('t = ')
%     disp(t)
%     disp('retained update time: ')
%     disp(next_update_time_in)
    if t >= next_update_time_in
   
%         disp('Updating control input at time = ')
%         disp(t)
%         disp('Current wrench: ')
%         disp(wrench')
        % Lambda = force directions we care about
        % ['Mx', 'My', 'Mz', 'X', 'Y', 'Z']
        % [should be 0, no, no, no, should be 0, should be small]
        lambda = diag([1 0 0 0 1 1]);
        
        d_xdes = -c.Kf*(lambda*(c.des_wrench-wrench));
%         disp('Wrench diff')
%         disp((c.des_wrench-wrench)')
%         disp('Desired Velocities')
%         disp(d_xdes')
    %     Vy = Kp*wrench(2);
        ideal_Vx = d_xdes(6);
        ideal_Vy = -d_xdes(5); %% BECAUSE +Y is actually down in EE frame

        AccelX = (ideal_Vx-X(2))/c.dt;
        AccelY = (ideal_Vy-X(4))/c.dt;
        
        if AccelX > c.maxAcc
            AccelX = c.maxAcc;
        elseif AccelX < -c.maxAcc
            AccelX = -c.maxAcc;
        end
        
        if AccelY > c.maxAcc
            AccelY = c.maxAcc;
        elseif AccelY < -c.maxAcc
            AccelY = -c.maxAcc;
        end
        
        Ax = AccelX;
        Ay = AccelY;
        next_update_time_out = t+c.dt;
%         disp('Updating next at: ')
%         disp(next_update_time_out)
    else
%         disp('Keeping same control input')
        ideal_Vx = X(2);
        ideal_Vy = X(4);
        AccelX = Ax; 
        AccelY = Ay;
        next_update_time_out = next_update_time_in;
    end
    

%      AccelX = Ax;
%      AccelY = Ay;
%      next_update_time_out = next_update_time_in;
%     disp('Actual Vx')
%     disp(X(2))
%     disp('Desired Vx')
%     disp(ideal_Vx)

end