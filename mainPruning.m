%{
Pruning Project: Admittance Control on End Effector

Flight code

Last modified by Hannah Kolano 6/23/2021
kolanoh@oregonstate.edu
%}

% clear all
close all

%% Set up parameters

% Geometry
p.r_branch = 0.005;     % Radius of the branch to cut, m

% Full representation of cutter
[p.top_x, p.top_y, p.bot_x, p.bot_y, p.joint_x, p.joint_y] = getprunercoords();

% Masses
p.m_branch = 0.010;       % WAG for mass of the branch

% Stiffness and Damping of branch
p.kx = 10;               % WAG for stiffness of branch
p.ky = 20;              % WAG. Stiffer in y direction b/c tree won't bend
p.b = .3;                % WAG (branch doesn't really oscillate)
p.ksquish = 100000;          % If the branch gets squished into the cutter
p.bsquish = .3;          % If the branch gets squished into the cutter

% Friction
p.mu_s = 0.6;           % WAG for wood/metal from Wikipedia
p.mu_k = 0.49;          % from Wikipedia https://en.wikipedia.org/wiki/Friction#Dry_friction

%% Simulate the system
% State: [x_cutter, xd_c, y_cutter, yd_c, x_branch, xd_b, y_branch, yd_b]
Cutter_X_init = -0.05;
Cutter_Vx_init = .05;
Cutter_Y_init = 0;
Cutter_Vy_init = -0.01;
X0 = [Cutter_X_init, Cutter_Vx_init, Cutter_Y_init, Cutter_Vy_init, 0.0001, 0, 0.0, 0];

[t_vec, X_vec] = simPruning(X0,p);

%% Plotting
% figure
% plot(t_vec, X_vec(5,:));
% xlabel('Time (s)')
% ylabel('Meters')
% title('Displacement')

%% Animate
exportVideo = false;
playbackRate = 1;
animationPruning(p,t_vec,X_vec,exportVideo,playbackRate);

