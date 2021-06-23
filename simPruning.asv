function [t_vec, X_vec] = simPruning(X0,p)  
    %{
    Simulation script for pruning simulation

    Inputs:
        X0: vector of initial conditions
        p: structure with system parameters

    Modified: Hannah Kolano 2021
    %}

    %% Initialization
    % Running time
    t_start = 0;
    t_end = .5;
    dt = 0.005;

    t_vec = t_start:dt:t_end;
    X_vec = zeros(length(X0), length(t_vec));
    sol_set = {};
    
    % States
    % 1: ball freefloating
    % 2: In contact with top blade
    % 3: In contact with bottom blade
    % 4: In contact with both blades
    p.state = 1;
    
    %% Function binding
    % Event functions
    free_event_fun = @(t,X)freeBranchEvent(t,X,p);  % When branch is freefloating
    hit_event_fun = @(t,X)contactBranchEvent(t,X,p); % When in contact with top cutter

    % Simulation tolerances/ ODE Options
    optionsFree = odeset(...                    % When branch is freefloating
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', free_event_fun);
    optionsHit = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-11, 'AbsTol', 1e-11, ...
        'Events', hit_event_fun, 'MaxStep', .03);

    % Bind dynamics function
    free_dyn_fun = @(t,X)freedyn(t,X,p);
    contact_dyn_fun = @(t,X)stoppeddyn(t,X,p);

    %% Iterate over all time
    while t_start < t_end
        % Simulate the dynamics over a time interval
        if p.state == 1 % (branch not in contact)
            sol = ode45(free_dyn_fun, [t_start,t_end], X0, optionsFree);
            disp('Contact! at t = ')
            disp(sol.x(end))
            p.state = 2;
        else % p.state == 2 (in contact with top blade)
            disp('Starting contact dynamics')
            X0(6) = X0(2); % Branch instantaneously matches velocity of cutter
            X0(8) = X0(4); 
            sol = ode45(contact_dyn_fun, [t_start,t_end], X0, optionsHit);
            disp('Came free! at t = ')
            disp(sol.x(end))
            p.state = 1;
        end

        % Concatenate solution sets
        sol_set = [sol_set, {sol}];
        % Setup t_start for the next ode45 call so it is at the end of the last call
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
    for idx = 1:length(sol_set)
        % This sets up a logical vector so we can perform logical indexing
        t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
        % Evaluate the idx solution structure only at the applicable times
        if any(t_sample_mask)
            X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
            % Assign the result to the correct indicies of the return state array
            X_vec(:,t_sample_mask) = X_eval;
        end
    end

end % simRDHT

%% Dynamics Functions

function dX = freedyn(t,X,p)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2)
    % p == parameters structure
%     Tau_ctrl = ctrl_fun(t,X);

    F_Kx = -p.kx*X(5)-p.b*X(6);
    F_Ky = -p.ky*X(7)-p.b*X(8);

   dX = zeros(length(X),1);
   dX(1) = X(2);
   dX(3) = X(4);
   dX(5) = X(6);
   dX(7) = X(8);
   
   dX(6) = F_Kx/p.m_branch;
   dX(8) = F_Ky/p.m_branch;
end % dynamics

function dX = stoppeddyn(t,X,p)
    X_C = X(1);  Y_C = X(3); X_B = X(5); Y_B = X(7);
       
    % Restoring forces to the branch
    F_Kx = -p.kx*X(5)-p.b*X(6);
    F_Ky = -p.ky*X(7)-p.b*X(8);
    F_K = sqrt(F_Ky^2+F_Kx^2);
    
    th_Fk = atan(F_Ky/F_Kx); % Angle of net restoring force (from horiz)
%     th_Fk = atan2(F_Kx, F_Ky);
    
%   translate the polyshapes for cutter and branch
    top_cutter = polyshape(p.top_x+X_C, p.top_y+Y_C);
    t1 = 0:.02:2*pi;
    branch_xs = p.r_branch.*cos(t1)+X_B;
    branch_ys = p.r_branch.*sin(t1)+Y_B;
    branchshape = polyshape(branch_xs, branch_ys);
   
    % Get intersection of cutter and branch
    poly_int = intersect(top_cutter, branchshape);
%     dyn_area = area(poly_int)
    [C_intx, C_inty] = centroid(poly_int);

    % Find angle of the normal
    dy = Y_B-C_inty;
    dx = X_B-C_intx;
    th_N = atan(dy/dx); % Angle of normal force

    % Angle to project F_K onto F_N vector 
    th_projection = th_N - th_Fk;

    F_N = F_K*cos(th_projection);
    F_Nx = F_N*cos(th_N);
    F_Ny = F_N*sin(th_N);

    dX = zeros(length(X),1);
   dX(1) = X(2);
   dX(3) = X(4);
   dX(5) = X(6);
   dX(7) = X(8);
   
   dX(6) = (F_Kx+F_Nx)/p.m_branch;
   dX(8) = (F_Ky+F_Ny)/p.m_branch;
   
end


%% Event Functions
function [eventVal, isterminal, direction] = freeBranchEvent(t,X,p)
    % Inputs
    % t: time, X: the state, p: parameters structure
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    
    % Position of center of cutter joint
    X_C = X(1);   Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    % translate the polyshape
    top_cutter = polyshape(p.top_x+X_C, p.top_y+Y_C);
    t3 = 0:.01:2*pi;
    branch = polyshape(p.r_branch.*cos(t3)+X_B, p.r_branch.*sin(t3)+Y_B);
    overlap_shape = intersect(top_cutter, branch);
    
%     [vertexid,~,~] = nearestvertex(top_cutter,X_B, Y_B);
%     dist_to_branch = sqrt((X_B-top_cutter.Vertices(vertexid,1))^2 + (Y_B-top_cutter.Vertices(vertexid,2))^2)-p.r_branch;

%     disp('Area of overlap')
%     disp(area(overlap_shape));
    eventVal = area(overlap_shape)-1e-8;
    isterminal = 1;     % stops the sim
    direction = 1;      % any direction
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
    t2 = 0:.01:2*pi;
    branch = polyshape(p.r_branch.*cos(t2)+X_B, p.r_branch.*sin(t2)+Y_B);
    
    overlap_shape = intersect(top_cutter, branch);

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    eventVal = area(overlap_shape)-1e-10;   % when spring at equilibrium distance...
%     eventVal = overlaps(top_cutter, branch);   % when spring at equilibrium distance...
    isterminal = 1;     % stops the sim
    direction = -1;      % any direction
end

