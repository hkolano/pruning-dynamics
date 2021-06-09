function [t_vec, X_vec] = simPruning(X0,p)  
    %{
    Simulation script for pruning simulation

    Inputs:
        X0: vector of initial conditions
        p: structure with system parameters

    Modified: Hannah Kolano 2021
    %}

    % Running time
    t_start = 0;
    t_end = 2;
    dt = 0.005;

    t_vec = t_start:dt:t_end;
    X_vec = zeros(length(X0), length(t_vec));
    sol_set = {};
    
%     free_event_fun = @(t,X)freeBallEvent(t,X,p);
%     hit_event_fun = @(t,X)contactBallEvent(t,X,p);

    % Simulation tolerances
    optionsFree = odeset(...
        'RelTol', 1e-5, 'AbsTol', 1e-5) %, ...
%         'Events', free_event_fun);
%     optionsFloor = odeset(...
%         'RelTol', 1e-5, 'AbsTol', 1e-5, ...
%         'Events', hit_event_fun);

    % Bind dynamics function
    free_dyn_fun = @(t,X)freedyn(t,X,p,ctlr_fun);
%     contact_dyn_fun = @(t,X)dyn_ballfloor(t,X,p,ctlr_fun);
    
    % States
    % 1: ball freefloating
    % 2: In contact with top blade
    % 3: In contact with bottom blade
    % 4: In contact with both blades
    p.state = 1;

    while t_start < t_end
        % Simulate the dynamics over a time interval
        if p.state == 1
            sol = ode45(free_dyn_fun, [t_start,t_end], X0, optionsFree);
            disp('Contact! at t = ')
            disp(sol.x(end))
            p.state = 2;
%         else % p.state == 2
%             sol = ode45(contact_dyn_fun, [t_start,t_end], X0, optionsFloor);
%             disp('Disconnected! at t = ')
%             disp(sol.x(end))
%             p.state = 1;
        end

        % Concatenate solution sets
        sol_set = [sol_set, {sol}];
        % Setup t_start for the next ode45 call so it is at the end of the
        % last call
        t_start = sol.x(end);
        % Set the initial conditions to the end of the last run
        if t_start == t_end
            X0 = [0 0 0 0 0 0 0 0];
        else
            X0 = sol.ye(:,end);
        end
    end
%
    % Loop to sample the solution structures and built X_vec
%     mask = zeros(1,length(t_vec));
    for idx = 1:length(sol_set)
        % This sets up a logical vector so we can perform logical indexing
        t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
%         disp('Is it empty?')
%         disp(any(t_sample_mask))
        % Evaluate the idx solution structure only at the applicable times
        if any(t_sample_mask)
            X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
            % Assign the result to the correct indicies of the return state array
            X_vec(:,t_sample_mask) = X_eval;
        end
    end

end % simRDHT

function dX = freedyn(t,X,p,ctrl_fun)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2)
    % p == parameters structure
%     Tau_ctrl = ctrl_fun(t,X);

    A = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/(p.Ip+p.Imotor)     -p.bp*p.r^2/(p.Ip+p.Imotor)    p.kp*p.r/(p.Ip+p.Imotor)             p.bp*p.r/(p.Ip+p.Imotor)               0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)     -p.bp*p.r^2/(p.Ip+p.Irod)];
    dX = A*X + [0; Tau_ctrl/(p.Ip+p.Imotor); 0; 0; 0; 0; 0; 0];
end % dynamics

%% Hybrid functions
function dX = dyn_ballfloor(t,X,p,ctrl_fun)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2)
    % p == parameters structure
    Tau_ctrl = ctrl_fun(t,X);

    M1 = p.mw2*p.A1/p.a + p.mpd*p.a/p.A1;
    M2 = p.mw2*p.A2/p.a + p.mpd*p.a/p.A2;
    kpaOVA1  = p.kp*p.a/p.A1;
    bpaOVA1  = p.bp*p.a/p.A1;
    kpaOVA2  = p.kp*p.a/p.A2;
    bpaOVA2  = p.bp*p.a/p.A2;

    if abs(X(3)) > p.strokelim
        k = 5000;
        b = 10;
    else
        k = 0;
        b = 0;
    end
    A = [0                 1                   0                           0                           0                           0                           0                   0; ...
        -p.kp*p.r^2/(p.Ip+p.Imotor)     -p.bp*p.r^2/(p.Ip+p.Imotor)    p.kp*p.r/(p.Ip+p.Imotor)             p.bp*p.r/(p.Ip+p.Imotor)              0                           0                           0                   0; ...
        0                   0                   0                           1                           0                           0                           0                   0; ...
        kpaOVA1*p.r/M1     bpaOVA1*p.r/M1     (-kpaOVA1-p.kh*p.A1/p.a-k)/M1  (-bpaOVA1-p.bf*p.A1/p.a-b)/M1  p.kh*p.A2/(p.a*M1)         0                           0                   0; ...
        0                   0                   0                           0                           0                           1                           0                   0; ...
        0                   0                   p.kh*p.A1/(p.a*M2)         0                           (-kpaOVA2-p.kh*p.A2/p.a)/M2  (-bpaOVA2-p.bf*p.A1/p.a)/M2  kpaOVA2*p.r/M2     bpaOVA2*p.r/M2; ...
        0                   0                   0                           0                           0                           0                           0                   1; ...
        0                   0                   0                           0                           p.kp*p.r/(p.Ip+p.Irod)      p.bp*p.r/(p.Ip+p.Irod)      -p.kp*p.r^2/(p.Ip+p.Irod)    -p.bp*p.r^2/(p.Ip+p.Irod)];
    dX = A*X + [0; Tau_ctrl/(p.Ip+p.Imotor); 0; 0; 0; 0; 0; 0];
    if dX(7) > 0
        dX(7) = 0;
    end
    if dX(8) > 0
        dX(8) = 0;
    end
end % dynamics

%% Hybrid functions
function [eventVal, isterminal, direction] = freeBallEvent(t,X,p)
    % Inputs
    % t: time, X: the state, p: parameters structure
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    height_rod = p.h-p.l_rod*sin(X(7));
    dist = height_rod-(p.obstacle_height+p.rball);
    eventVal = dist;   % when spring at equilibrium distance...
    isterminal = 1;     % stops the sim
    direction = -1;      % any direction
end
%
function [eventVal, isterminal, direction] = contactBallEvent(t,X,p)
    % t: time, X: the state, p: parameters structure
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    height_rod = p.h-p.l_rod*sin(X(7));
    dist = height_rod-(p.obstacle_height+p.rball);
    eventVal = dist;   % when spring at equilibrium distance...
    isterminal = 1;     % stops the sim
    direction = 1;      % any direction
end

