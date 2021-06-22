%{
Pruning Project: Admittance Control on End Effector

Flight code

Last modified by Hannah Kolano 6/10/2021
kolanoh@oregonstate.edu
%}

% clear all
close all

%% Set up parameters

% Geometry
p.r_branch = 0.005;     % Radius of the branch to cut, m
% assuming cutters represented as lines
% p.l_cutter = 0.05;      % Length of the cutter, m   
% p.th = deg2rad(30);     % Angle of cutter, rad
% Full representation of cutter
[p.top_x, p.top_y, p.bot_x, p.bot_y, p.joint_x, p.joint_y] = getprunercoords();

% Masses
p.m_branch = 0.010;       % WAG for mass of the branch

% Stiffness and Damping
p.kx = 10;               % WAG for stiffness of branch
p.ky = 20;              % WAG. Stiffer in y direction b/c tree won't bend

% Damping
p.b = .3;                % WAG (branch doesn't really oscillate)

%% Simulate the system
% State: [x_cutter, xd_c, y_cutter, yd_c, x_branch, xd_b, y_branch, yd_b]
X0 = [-.05, .05, -.005, 0, 0.01, 0, 0.0, 0];

[t_vec, X_vec] = simPruning(X0,p);

%% Plotting
% figure
% plot(t_vec, X_vec(5,:));
% xlabel('Time (s)')
% ylabel('Meters')
% title('Displacement')

%% Animate
exportVideo = false;
playbackRate = .5;
animationPruning(p,t_vec,X_vec,exportVideo,playbackRate);

