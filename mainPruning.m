%{
Pruning Project: Admittance Control on End Effector

Flight code

Last modified by Hannah Kolano 6/9/2021
kolanoh@oregonstate.edu
%}

clear all

%% Set up parameters

% Geometry
p.r_branch = 0.005;     % Radius of the branch to cut, m
% assuming cutters represented as lines
p.l_cutter = 0.05;      % Length of the cutter, m            

% Masses
m_branch = 0.010;       % WAG for mass of the branch

% Stiffness and Damping
p.kx = 10;               % WAG for stiffness of branch
p.ky = 20;              % WAG. Stiffer in y direction b/c tree won't bend

% Damping
p.b = 1;                % WAG (branch doesn't oscillate forever)

%% Simulate the system
% State: [x_cutter, y_cutter, theta_cutter, x_branch, y_branch]
X0 = [-.1, 0, deg2rad(30), 0, 0];

[t_vec, X_vec] = simPruning(X0,p);

%% Plotting
figure
% plot(t_vec, X_vec(1,:))
plot(t_vec, X_vec(:,1));
hold on
plot(t_vec, X_vec(:,7))
% plot(t_vec, 5*sin(t_vec))
legend('Theta1', 'Theta2')
xlabel('Time (s)')
ylabel('Radians')
title('Angular Displacement')

exportVideo = false;
playbackRate = 1;
animationPruning(p,t_vec,X_vec,exportVideo,playbackRate);

