%{
Pruning Project: Admittance Control on End Effector

Flight code

Last modified by Hannah Kolano 6/30/2021
kolanoh@oregonstate.edu
%}

% clear all
close all

addpath('C:\Users\hkolano\Documents\GitHub\ModernRobotics\packages\MATLAB\mr')

p.runtime = .5;
p.dt = 0.001;

%% Set up geometry parameters p

% Geometry
p.r_branch = 0.005;     % Radius of the branch to cut, m

% Full representation of cutter
[p.top_x, p.top_y, p.bot_x, p.bot_y, p.joint_x, p.joint_y] = getprunercoords();

% Masses
p.m_branch = 0.010;       % WAG for mass of the branch

% Object polyshapes
p.top_shape = polyshape(p.top_x, p.top_y);
p.bottom_shape = polyshape(p.bot_x, p.bot_y);
t1 = 0:.02:2*pi;
branch_xs = p.r_branch.*cos(t1);
branch_ys = p.r_branch.*sin(t1);
p.branch_shape = polyshape(branch_xs, branch_ys);

% Transform actual end effector frame (right/down/forward) to what I thought the end effector
% frame was (forward/up/left)
T_ee_matlabee = [0 0 1 0;
                0 -1 0 0;
                1 0 0 0;
                0 0 0 1];
% Transform from the end effector frame to the cut point
T_matlabee_cutpoint= [1 0 0 .09707;
                0 1 0 -.005; 
                0 0 1 .09286;
                0 0 0 1]; 
% Transform from the cut point to the cutter "joint" (0 in the polyshape)
T_cutpoint_cutjoint = [1 0 0 -.0137;
            0 1 0 .00516; 
            0 0 1 0;
            0 0 0 1]; 
p.T_ee_cutter = T_ee_matlabee*T_matlabee_cutpoint*T_cutpoint_cutjoint;

%% Set up Physics Parameters p
% Stiffness and Damping of branch
p.kx = 10;               % WAG for stiffness of branch
p.ky = 20;              % WAG. Stiffer in y direction b/c tree won't bend
p.b = .3;                % WAG (branch doesn't really oscillate)
p.ksquish = 100000;          % If the branch gets squished into the cutter
% p.bsquish = .3;          % If the branch gets squished into the cutter

% Friction
p.mu_s = 0.6;           % WAG for wood/metal from Wikipedia
p.mu_k = 0.49;          % from Wikipedia https://en.wikipedia.org/wiki/Friction#Dry_friction

%% Set up Controller Parameters c
% Controller gains (Kf = force feedback gain)
c.Kf = .5;

% Desired wrench at EE
% ['Mx', 'My', 'Mz', 'X', 'Y', 'Z']
% [should be 0, no, no, no, should be 0, zhould be small]
c.des_wrench = [0 0 0 0 0 -0.02]';
p.next_update_time = -.001;
c.dt = 0.008; % Control loop
c.maxAcc = .5; % maximum acceleration of EE

%% Anonymous functions
ctlr_fun = @(t,next_t,wrench,X) AdmitCtlrPruning(t,next_t,c,wrench,X);

%% Simulate the system
% State: [x_cutter, xd_c, y_cutter, yd_c, x_branch, xd_b, y_branch, yd_b]
Cutter_X_init = -0.03;
Cutter_Vx_init = 0.05;
Cutter_Y_init = 0.005;
Cutter_Vy_init = 0.0;
X0 = [Cutter_X_init, Cutter_Vx_init, Cutter_Y_init, Cutter_Vy_init, 0.0001, 0, 0.0, 0];

[t_vec, X_vec] = simPruning(X0,p,ctlr_fun);


%% Animate
exportVideo = false;
playbackRate = .5;
animationPruning(p,t_vec,X_vec,exportVideo,playbackRate);

%% Getting Forces
% Set up force vector
F_vec = zeros(8, length(t_vec));
wrench_vec = zeros(6, length(t_vec));
for i = 1:length(t_vec)
    x_state = X_vec(:,i);
    top_cutter = translate(p.top_shape, x_state(1), x_state(3));
    bottom_cutter = translate(p.bottom_shape, x_state(1), x_state(3));
    branch = translate(p.branch_shape, x_state(5), x_state(7));

    top_hit = overlaps(top_cutter, branch);
    bot_hit = overlaps(bottom_cutter, branch);
    
    % Find intersect between the cutter and the branch
    if top_hit == 1 && bot_hit == 0
        [forces, wrenches] = get_forces(p, x_state, top_cutter, branch);
    elseif bot_hit == 1 && top_hit == 0
        [forces, wrenches] = get_forces(p, x_state, bottom_cutter, branch);
    elseif top_hit == 1 && bot_hit == 1
        [forces, wrenches] = get_forces_attached(p, x_state, top_cutter, bottom_cutter, branch);
    else
        forces = zeros(1,8);
        wrenches = zeros(1,6);
    end
    
    F_vec(:,i) = forces';
    wrench_vec(:,i) = wrenches';
%     
end

%% Plot X Forces
% figure
% plot(t_vec, F_vec(1,:))
% hold on
% plot(t_vec, F_vec(3,:))
% plot(t_vec, F_vec(5,:))
% plot(t_vec, F_vec(7,:))
% plot(t_vec, F_vec(1,:)+F_vec(3,:)+F_vec(5,:)+F_vec(7,:), 'k')
% xlabel('Time (s)')
% ylabel('X forces (N)')
% legend('Restoring X', 'Top Normal X', 'Bottom Normal X', 'Friction X', 'Total')
% title('Net X Forces')

%% Plot Y Forces
% figure
% plot(t_vec, F_vec(2,:))
% hold on
% plot(t_vec, F_vec(4,:))
% plot(t_vec, F_vec(6,:))
% plot(t_vec, F_vec(8,:))
% plot(t_vec, F_vec(2,:)+F_vec(4,:)+F_vec(6,:)+F_vec(8,:), 'k')
% xlabel('Time (s)')
% ylabel('Y forces (N)')
% legend('Restoring Y', 'Top Normal Y', 'Bottom Normal Y', 'Friction Y', 'Total')
% title('Net Y Forces')

%% Plot velocities
figure
plot(t_vec, X_vec(2,:))
hold on
plot(t_vec, X_vec(4,:))
plot(t_vec, X_vec(6,:))
plot(t_vec, X_vec(8,:))
xlabel('Time (s)')
ylabel('Velocities (m/s)')
legend('X vels', 'Y vels', 'Branch X', 'Branch Y')

%% Plot Wrenches
figure
plot(t_vec, wrench_vec(1,:))
hold on
for i = 2:6
   plot(t_vec, wrench_vec(i,:))
end
xlabel('Time (s)')
legend('Mx', 'My', 'Mz', 'X', 'Y', 'Z')
title('Wrench At Sensor')

%% FUNCTIONS
function [Forces, wrench] = get_forces_attached(p, x_state, top_cutter, bottom_cutter, branch)
    % Get restoring Forces of branch
    [F_Kx, F_Ky, ~, ~] = getRestoringForces(p, x_state);
    
    poly_int_top = intersect(top_cutter, branch);
    poly_int_bottom = intersect(bottom_cutter, branch);
    [C_intx_top, C_inty_top] = centroid(poly_int_top);
    [C_intx_bot, C_inty_bot] = centroid(poly_int_bottom);
%         plot([C_intx, x_state(5)], [C_inty, x_state(7)], 'b', 'LineWidth', 2)

    % Find angle of the normal
    dy_top = C_inty_top-x_state(7);
    dx_top = C_intx_top-x_state(5);
    th_T = atan2(dy_top,dx_top); % Angle of normal force from top
    
    dy_bottom = C_inty_bot-x_state(7);
    dx_bottom = C_intx_bot-x_state(5);
    th_B = atan2(dy_bottom, dx_bottom); % Angle of normal force from bottom

    th_Tstar = th_T-pi;
    th_Bstar = pi+th_B;
%     F_NB = (F_Kx - F_Ky)/(cos(th_B+pi/2) - sin(th_B+pi/2));
%     F_NT = (F_Ky*cos(th_B+pi/2) - F_Kx*sin(th_B+pi/2))/(cos(th_T - pi/2)*(cos(th_B + pi/2) - sin(th_B + pi/2)));
    F_NT = (-F_Ky*cos(th_Bstar) + F_Kx*sin(th_Bstar))/(sin(th_Tstar)*cos(th_Bstar)-cos(th_Tstar)*sin(th_Bstar));
    F_NB = (-F_Kx-F_NT*cos(th_Tstar))/cos(th_Bstar);
 
    F_NTx = F_NT*cos(th_Tstar);
    F_NTy = F_NT*sin(th_Tstar);
    
    F_NBx = F_NB*cos(th_Bstar);
    F_NBy = F_NB*sin(th_Bstar);
    
    Forces = [F_Kx, F_Ky, F_NTx, F_NTy, F_NBx, F_NBy, 0, 0];
    cutter_forces = [-F_NTx, -F_NBx, -F_NTy, -F_NBy, 0, 0, 0, 0];
    %     [CPtopX, CPbottomX, CPtopY, CPbottomY]
    centpoints = [C_intx_top, C_intx_bot, C_inty_top, C_inty_bot];
    wrench = getForceTorqueMeasurement(p, x_state, cutter_forces, centpoints);
end

function [Forces, wrench] = get_forces(p, x_state, cutter, branch)
    % Get restoring forces of branch
    [F_Kx, F_Ky, F_K, th_Fk] = getRestoringForces(p, x_state);
    
        poly_int = intersect(cutter, branch);
        [C_intx, C_inty] = centroid(poly_int);
%         plot([C_intx, x_state(5)], [C_inty, x_state(7)], 'b', 'LineWidth', 2)
        
        % Find angle of the normal
        dy = C_inty-x_state(7);
        dx = C_intx-x_state(5);
        th_N = atan2(dy,dx); % Angle of normal force

        % Angle to project F_K onto F_N vector 
        th_projection = th_Fk-th_N;
%         disp([th_Fk, th_Fk2, th_N, th_projection]);

        F_N = -F_K*cos(th_projection); % Restoring force perp. to surface 
        F_rem = F_K*sin(th_projection); % Restoring force parallel to surface
        F_Nx = F_N*cos(th_N);
        F_Ny = F_N*sin(th_N);
        
        relVelX = x_state(6)-x_state(2);
        relVelY = x_state(8)-x_state(4);
        th_relV_ang = atan2(relVelY, relVelX);
        relVel_mag = sqrt(relVelY^2 + relVelX^2);
        Vel_B_Par = relVel_mag*sin(th_N-th_relV_ang);
        Vel_B_ParX = Vel_B_Par*sin(th_N);
        Vel_B_ParY = -Vel_B_Par*cos(th_N);
        
          if relVel_mag < 0.001 % If branch isn't moving, add stiction
%                disp('Not moving; adding stiction')
                max_Ff = abs(p.mu_s*F_N);
%                 disp(F_rem);
                F_f = max_Ff*(-0.0216*(F_rem/max_Ff)^5 - 1E-04*(F_rem/max_Ff)^4 + 0.2475*(F_rem/max_Ff)^3 + 0.0004*(F_rem/max_Ff)^2 - 1.136*(F_rem/max_Ff) - 0.0002);
           else % If branch is moving, apply kinetic friction
        %        disp('Branch is moving; adding kinetic friction')
%                F_f = 0;
               F_f = abs(p.mu_k*F_N);
           end % if stiction

           F_fx = abs(F_f*sin(th_N))*sign(-Vel_B_ParX);
           F_fy = abs(F_f*cos(th_N))*sign(-Vel_B_ParY);
       
       if F_Ny < 0 % hitting top cutter
           Forces = [F_Kx, F_Ky, F_Nx, F_Ny, 0, 0, F_fx, F_fy];
           cutter_forces = [-F_Nx, 0, -F_Ny, 0, -F_fx, 0, -F_fy, 0];
           centpoints = [C_intx, 0, C_inty, 0];
       else % hitting bottom cutter
           Forces = [F_Kx, F_Ky, 0, 0, F_Nx, F_Ny, F_fx, F_fy];
           cutter_forces = [0, -F_Nx, 0, -F_Ny, 0, -F_fx, 0, -F_fy];
           centpoints = [0, C_intx, 0, C_inty];
       end
       
       wrench = getForceTorqueMeasurement(p, x_state, cutter_forces, centpoints);
           
    end

