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
    
    free_event_fun = @(t,X)freeBranchEvent(t,X,p);
    hit_event_fun = @(t,X)contactBranchEvent(t,X,p);

    % Simulation tolerances
    optionsFree = odeset(...
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', free_event_fun);
    optionsHit = odeset(...
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', hit_event_fun);

    % Bind dynamics function
    free_dyn_fun = @(t,X)freedyn(t,X,p);
    contact_dyn_fun = @(t,X)stoppeddyn(t,X,p);
    
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
        else % p.state == 2
            sol = ode45(contact_dyn_fun, [t_start,t_end], X0, optionsHit);
%             disp('Disconnected! at t = ')
%             disp(sol.x(end))
            p.state = 1;
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

function dX = freedyn(t,X,p)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2)
    % p == parameters structure
%     Tau_ctrl = ctrl_fun(t,X);

   dX = zeros(length(X),1);
   dX(1) = X(2);
   dX(3) = X(4);
   dX(5) = X(6);
   dX(7) = X(8);
   
   dX(6) = (-p.kx*X(5)-p.b*X(6))/p.m_branch;
   dX(8) = (-p.ky*X(7)-p.b*X(8))/p.m_branch;
end % dynamics

function dX = stoppeddyn(t,X,p)

    X_C = X(1);  Y_C = X(3); X_B = X(5); Y_B = X(7);
    % translate the polyshape
%     top_cutter = polyshape(p.top_x+X_C, p.top_y+Y_C);
%     t1 = 0:.002:2*pi;
%     branch = polyshape(p.r_branch*cos(t1)+X_B, p.r_branch*sin(t1)+Y_B);
%     
%     poly_int = intersect(top_cutter, branch);
%     [C_intx, C_inty] = centroid(poly_int)
%     
%     [vertexid,~,~] = nearestvertex(branch,C_intx, C_inty);
%     dy = C_inty-branch.Vertices(vertexid,2);
%     dx = C_intx-branch.Vertices(vertexid,1);
%     ang = atan(dy/dx)

    dX = zeros(length(X),1);
   dX(1) = X(2);
   dX(3) = X(4);
   
   if sign((-p.kx*X(5)-p.b*X(6))/p.m_branch) ~= sign(X(2))
       dX(6) = (-p.kx*X(5)-p.b*X(6))/p.m_branch;
   else
       dX(5) = X(2);
   end
   if sign((-p.ky*X(7)-p.b*X(8))/p.m_branch) ~= sign(X(4))
       dX(8) = (-p.ky*X(7)-p.b*X(8))/p.m_branch;
   else
       dX(7) = X(4);
   end

end

%% Hybrid functions
function dX = dyn_ballfloor(t,X,p,ctrl_fun)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2)
    % p == parameters structure

end % dynamics

%% Hybrid functions
function [eventVal, isterminal, direction] = freeBranchEvent(t,X,p)
    % Inputs
    % t: time, X: the state, p: parameters structure
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    
    % Position of center of cutter joint
    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    % translate the polyshape
    top_cutter = polyshape(p.top_x+X_C, p.top_y+Y_C);
    
    [vertexid,~,~] = nearestvertex(top_cutter,X_B, Y_B);
    dist_to_branch = sqrt((X_B-top_cutter.Vertices(vertexid,1))^2 + (Y_B-top_cutter.Vertices(vertexid,2))^2)-p.r_branch;

%     height_rod = p.h-p.l_rod*sin(X(7));
%     dist = height_rod-(p.obstacle_height+p.rball);
    eventVal = dist_to_branch-.00;   % when spring at equilibrium distance...
    isterminal = 1;     % stops the sim
    direction = -1;      % any direction
end
%
function [eventVal, isterminal, direction] = contactBranchEvent(t,X,p)
    % t: time, X: the state, p: parameters structure
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
        % Position of center of cutter joint
    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    % translate the polyshape
    top_cutter = polyshape(p.top_x+X_C, p.top_y+Y_C);
    t1 = 0:.002:2*pi;
    branch = polyshape(p.r_branch*cos(t1)+X_B, p.r_branch*sin(t1)+Y_B);
    
    poly_int = intersect(top_cutter, branch);
    [C_intx, C_inty] = centroid(poly_int)
    
    eventVal = 1;   % when spring at equilibrium distance...
    isterminal = 0;     % stops the sim
    direction = 1;      % any direction
end

