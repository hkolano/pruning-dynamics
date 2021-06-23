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

%% Simulate the system
% State: [x_cutter, xd_c, y_cutter, yd_c, x_branch, xd_b, y_branch, yd_b]
X0 = [-.05, .05, -.005, 0, 0.0001, 0, 0.0, 0];

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

