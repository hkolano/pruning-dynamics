function [t_vec, X_vec] = simPruning(X0,p, ctlr_fun)  
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
    t_end = p.runtime;
    dt = p.dt;
    
    global next_update_time 
    next_update_time = -0.01;
    global Ax
    Ax = 0;
    global Ay
    Ay = 0;

    sol_set = {};
    
    p.top_shape = polyshape(p.top_x, p.top_y);
    p.bottom_shape = polyshape(p.bot_x, p.bot_y);
    t1 = 0:.02:2*pi;
    branch_xs = p.r_branch.*cos(t1);
    branch_ys = p.r_branch.*sin(t1);
    p.branch_shape = polyshape(branch_xs, branch_ys);
    
    % States
    % 1: branch freefloating
    % 2: In contact with top blade/sticking
    % 3: In contact with top blade/sliding
    % 4: In contact with bottom blade/sticking
    % 5: In contact with bottom blade/sliding
    % 6: In contact with both blades
    state = 1;
    
    %% Function binding
    % Event functions
    free_event_fun = @(t,X)freeBranchEvent(t,X,p);  % When branch is freefloating
    top_sticking_event_fun = @(t,X)contactTopStickingEvent(t,X,p); % When in contact with top cutter
    bot_sticking_event_fun = @(t,X)contactBottomStickingEvent(t,X,p);
    top_sliding_event_fun = @(t,X)contactTopSlidingEvent(t,X,p);
    bot_sliding_event_fun = @(t,X)contactBottomSlidingEvent(t,X,p);
    both_event_fun = @(t,X)bothcontactEvent(t,X,p);

    % Simulation tolerances/ ODE Options
    optionsFree = odeset(...                    % When branch is freefloating
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', free_event_fun, 'MaxStep', .008);     % Max step = control loop
    optionsTopSticking = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', top_sticking_event_fun, 'MaxStep', .001);
    optionsTopSliding = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', top_sliding_event_fun, 'MaxStep', .0005);
    optionsBottomSticking = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', bot_sticking_event_fun, 'MaxStep', .001);
    optionsBottomSliding = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', bot_sliding_event_fun, 'MaxStep', .001);
    optionsBoth = odeset(...
        'RelTol', 1e-9, 'AbsTol', 1e-9, ...
        'Events', both_event_fun, 'MaxStep', .0001);

    % Bind dynamics function
    free_dyn_fun = @(t,X)freedyn(t,X,p, ctlr_fun);
    top_sticking_dyn_fun = @(t,X)stickingdyn(t,X,p,p.top_shape, 0, ctlr_fun);
    bottom_sticking_dyn_fun = @(t,X)stickingdyn(t,X,p,p.bottom_shape, 1, ctlr_fun);
    top_sliding_dyn_fun = @(t,X)slidingdyn(t,X,p,p.top_shape, 0, ctlr_fun);
    bottom_sliding_dyn_fun = @(t,X)slidingdyn(t,X,p,p.bottom_shape, 1, ctlr_fun);
    both_dyn_fun = @(t,X)bothhitdyn(t,X,p, ctlr_fun);

    %% Iterate over all time
    while t_start < t_end
        % Simulate the dynamics over a time interval
        if state == 1 % (branch not in contact)
            disp('Starting Free dynamics')
            sol = ode45(free_dyn_fun, [t_start,t_end], X0, optionsFree);
            disp('Contact at t = ')
            disp(sol.x(end))
            if sol.ie == 2 % If trigger top contact in event
                state = 3; % Change to top sliding dynamics
            elseif sol.ie == 3
                state = 5; % Change to bottom sliding dynamics
            elseif sol.ie == 1
                state = 6;
            else
                disp('Improper event index.')
            end
        elseif state == 2 % state == 2 (sticking to top blade)
            disp('Contact with top -- sticking')
%             X0 = calc_instant_contact_vels(p, X0, p.top_shape);
            sol = ode45(top_sticking_dyn_fun, [t_start,t_end], X0, optionsTopSticking);
            disp('Change at t = ')
            disp(sol.x(end))
%             disp(sol.ie)
            if sol.ie == 1 % to Free
                state = 1; 
            elseif sol.ie == 2 % to Top/Sliding
                state = 3; 
            elseif sol.ie == 3
                state = 6;  % to Both
            elseif any(ismember(sol.ie, 1)) % If triggers events 1 and 2 at same time
                state = 1;
            else
                disp('Improper event index.')
            end
        elseif state == 3 % state == 2 (sliding on top blade)
            disp('Contact with top -- sliding')
            X0 = calc_instant_contact_vels(p, X0, p.top_shape);
            sol = ode45(top_sliding_dyn_fun, [t_start,t_end], X0, optionsTopSliding);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1
                state = 1; % to Free
            elseif sol.ie == 2
                state = 2; % to Top/Sticking
            elseif sol.ie == 3
                state = 6;
            else
                disp('Improper event index.')
            end
        elseif state == 4 %(sticking to bottom blade)
            disp('Contact with bottom -- sticking')
%             X0 = calc_instant_contact_vels(p, X0, p.bottom_shape); 
            sol = ode45(bottom_sticking_dyn_fun, [t_start,t_end], X0, optionsBottomSticking);
            disp('Change at t = ')
            disp(sol.x(end))
%             disp(sol.ie)
            if sol.ie == 1
                state = 1; % to Free
            elseif sol.ie == 2
                state = 5; % to Bottom/Sliding
            elseif sol.ie == 3
                state = 6; % to both
            else
                disp('Improper event index.')
            end
        elseif state == 5 %(sliding on bottom blade)
            disp('Contact with bottom -- sliding')
            X0 = calc_instant_contact_vels(p, X0, p.bottom_shape);
            sol = ode45(bottom_sliding_dyn_fun, [t_start,t_end], X0, optionsBottomSliding);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1
                state = 1; % to Free
            elseif sol.ie == 2
                state = 4; % to Bottom/sticking
            elseif sol.ie == 3
                state = 6;
            else
                disp('Improper event index.')
            end
        else % state = 6
            disp('Contact with both!')
            matching_vels_state = X0;
            matching_vels_state(6) = X0(2);
            matching_vels_state(8) = X0(4);
            sol = ode45(both_dyn_fun, [t_start, t_end], matching_vels_state, optionsBoth);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1 % no longer in contact with bottom (to top sticking)
                state = 2;
            elseif sol.ie == 2 % no longer in contact with top (to bottom sticking)
                state = 4;
            else
                disp('Improper event index or end of sim')
            end
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
    t_vec = 0:dt:t_start;
    X_vec = zeros(length(X0), length(t_vec));
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

function dX = freedyn(t,X,p, ctlr_fun)
    % t == time
    % X == the state (theta1, dtheta1, x1, dx1, x2, dx2, theta2, dtheta2)
    % p == parameters structure
    % ctlr_fun == controller function
    global next_update_time

    F_Kx = -p.kx*X(5)-p.b*X(6);
    F_Ky = -p.ky*X(7)-p.b*X(8);
    
    [newAx, newAy, new_update_time] = ctlr_fun(t, next_update_time, [0 0 0 0 0 0]', X);
   next_update_time = new_update_time; 


   dX = zeros(length(X),1);
   dX(1) = X(2);
   dX(3) = X(4);
   dX(5) = X(6);
   dX(7) = X(8);
   
   dX(2) = newAx;
   dX(4) = newAy;
   dX(6) = F_Kx/p.m_branch;
   dX(8) = F_Ky/p.m_branch;
end % dynamics

function dX = stickingdyn(t,X,p,shape, is_bottom, ctlr_fun)

    global next_update_time
    
    dX = zeros(length(X),1);
    dX(1) = X(2);  dX(3) = X(4);  dX(5) = X(6);  dX(7) = X(8);
    
    % Restoring forces to the branch
    [F_Kx, F_Ky, F_K, th_Fk] = getRestoringForces(p, X);
    
    % Calculate the current Normal angle th_N
    [th_N, dist_to_overlap, C_intX, C_intY] = calc_normal_angle(p, X, shape);

    % Angle to project F_K onto F_N vector 
    th_projection = th_Fk-th_N;

    % Get normal force
    F_N = -F_K*cos(th_projection);
    F_rem = F_K*sin(th_projection); % Net remaining force, tangent to surface
    
    % Squish force (to account for concave shapes)
    squish_dist = p.r_branch-dist_to_overlap; % 
    if squish_dist > 0
        F_N = F_N - squish_dist^2*p.ksquish;
    end
    
    % Separate Normal forces into components
    F_Nx = F_N*cos(th_N);
    F_Ny = F_N*sin(th_N);
   
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
    th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);
    Vel_B_Par = relVel_mag*sin(th_N-th_relV_ang);
    Vel_B_ParX = Vel_B_Par*sin(th_N);
    Vel_B_ParY = -Vel_B_Par*cos(th_N);
    
    max_Ff = abs(p.mu_s*F_N);

    F_f = max_Ff*(-0.0216*(F_rem/max_Ff)^5 - 1E-04*(F_rem/max_Ff)^4 + 0.2475*(F_rem/max_Ff)^3 + 0.0004*(F_rem/max_Ff)^2 - 1.136*(F_rem/max_Ff) - 0.0002);
     
%     F_f = 0;
%    disp(F_f)
   F_fx = abs(F_f*sin(th_N))*sign(-Vel_B_ParX);
   F_fy = abs(F_f*cos(th_N))*sign(-Vel_B_ParY);
%     F_fx = 0;
%     F_fy = 0;

    cutter_forces = zeros(1, 8); % [FNtopX, FNbottomX, FNtopY, FNbottomY, FftopX, FfbottomX, FftopY,
%    FfbottomY];
    % This is REALLY STUPID trying to avoid if statements
    % Normal X force (if top, index 1, if bottom, index 2)
    cutter_forces(is_bottom+1) = -F_Nx;
    cutter_forces(is_bottom+3) = -F_Ny;
    cutter_forces(is_bottom+5) = -F_fx;
    cutter_forces(is_bottom+7) = -F_fy;
    
%     [CPtopX, CPbottomX, CPtopY, CPbottomY]
    centpoints = zeros(1,4);
    centpoints(is_bottom+1) = C_intX;
    centpoints(is_bottom+3) = C_intY;  
    
   wrench = getForceTorqueMeasurement(p, X, cutter_forces, centpoints);
   
   [newAx, newAy, new_update_time] = ctlr_fun(t, next_update_time, wrench, X);
   next_update_time = new_update_time;
   
%    dX(1) = newVx;
   dX(2) = newAx;
%    dX(3) = newVy;
   dX(4) = newAy;
   dX(6) = (F_Kx+F_Nx+F_fx)/p.m_branch;
   dX(8) = (F_Ky+F_Ny+F_fy)/p.m_branch;
   
end

function dX = slidingdyn(t,X,p,shape, is_bottom, ctlr_fun)

    % Get next time the controller will update
    global next_update_time
    
    % Restoring forces to the branch
    [F_Kx, F_Ky, F_K, th_Fk] = getRestoringForces(p, X);
    
    % Calculate the current Normal angle th_N
    [th_N, dist_to_overlap, C_intX, C_intY] = calc_normal_angle(p, X, shape);

    % Angle to project F_K onto F_N vector 
    th_projection = th_Fk-th_N;

    % Get normal forces
    F_N = -F_K*cos(th_projection);
%     F_rem = F_K*sin(th_projection); % Net remaining force, tangent to surface
    
    % Squish force (to account for concave shapes)
    squish_dist = p.r_branch-dist_to_overlap; % 
    if squish_dist > 0
        F_N = F_N - squish_dist^2*p.ksquish;
    end
    
    % Separate Normal forces into components
    F_Nx = F_N*cos(th_N);
    F_Ny = F_N*sin(th_N);

    dX = zeros(length(X),1);
    dX(1) = X(2);  dX(3) = X(4);  dX(5) = X(6);  dX(7) = X(8);
   
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
    th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);
    Vel_B_Par = relVel_mag*sin(th_N-th_relV_ang);
    Vel_B_ParX = Vel_B_Par*sin(th_N);
    Vel_B_ParY = -Vel_B_Par*cos(th_N);

    F_f = abs(p.mu_k*F_N);
%    disp(F_f)
    F_fx = abs(F_f*sin(th_N))*sign(-Vel_B_ParX);
    F_fy = abs(F_f*cos(th_N))*sign(-Vel_B_ParY);
%     F_fx = 0;
%     F_fy = 0;

    cutter_forces = zeros(1, 8); % [FNtopX, FNbottomX, FNtopY, FNbottomY, FftopX, FfbottomX, FftopY,
%    FfbottomY];
    % This is REALLY STUPID trying to avoid if statements
    % Normal X force (if top, index 1, if bottom, index 2)
    cutter_forces(is_bottom+1) = -F_Nx;
    cutter_forces(is_bottom+3) = -F_Ny;
    cutter_forces(is_bottom+5) = -F_fx;
    cutter_forces(is_bottom+7) = -F_fy;
%     disp(cutter_forces)
    
    %     [CPtopX, CPbottomX, CPtopY, CPbottomY]
    centpoints = zeros(1,4);
    centpoints(is_bottom+1) = C_intX;
    centpoints(is_bottom+3) = C_intY;
%     disp(centpoints);
    
    wrench = getForceTorqueMeasurement(p, X, cutter_forces, centpoints);
%     disp(wrench')
   
   [newAx, newAy, new_update_time] = ctlr_fun(t, next_update_time, wrench, X);
   next_update_time = new_update_time;
%    dX(1) = newVx;
   dX(2) = newAx;
%    dX(3) = newVy;
   dX(4) = newAy;
   dX(6) = (F_Kx+F_Nx+F_fx)/p.m_branch;
   dX(8) = (F_Ky+F_Ny+F_fy)/p.m_branch;
   
end

function dX = bothhitdyn(t,x_state,p, ctlr_fun)
%     disp(t)
    global next_update_time
    
    dX = zeros(length(x_state),1);
    dX(1) = x_state(2);  dX(3) = x_state(4);  dX(7) = x_state(4);
    if x_state(2) > 0
        dX(5) = x_state(2);
    else 
        dX(5) = x_state(7);
    end
       
    [F_Kx, F_Ky, ~, ~] = getRestoringForces(p, x_state);

    % Get centerpoints and angles of the normal contact
%         disp('Calc Normals')
    [centpoints, th_T, th_B, ~, ~] = calcNormalsBothHit(p, x_state);
        
    % Get normal forces that oppose the restoring force
    [F_NTx, F_NTy, F_NBx, F_NBy] = getNormalForcesBothBlades(th_T, th_B, F_Kx, F_Ky);
    
    cutter_forces = [-F_NTx, -F_NBx, -F_NTy, -F_NBy, 0, 0, 0, 0];
    wrench = getForceTorqueMeasurement(p, x_state, cutter_forces, centpoints);
%     disp(wrench')
    
   [newAx, newAy, new_update_time] = ctlr_fun(t, next_update_time, wrench, x_state);
   next_update_time = new_update_time; 
%    dX(1) = newVx;
   dX(2) = newAx;
%    dX(3) = newVy;
   dX(4) = newAy;
   dX(6) = (F_Kx+F_NTx+F_NBx)/p.m_branch;  
   dX(8) = (F_Ky+F_NTy+F_NBy)/p.m_branch;
end


%% Helper Functions
function [th_N, dist_to_overlap, C_intx, C_inty] = calc_normal_angle(p, x_state, shape)
    % Translate cutter and branch
    cutter = translate(shape, x_state(1), x_state(3));
    branchshape = translate(p.branch_shape, x_state(5), x_state(7));
    % Find center of the overlapping shape
    poly_int = intersect(cutter, branchshape);
    [C_intx, C_inty] = centroid(poly_int);
    
    % Distance vectors from centroid of overlap to center of branch
    dy = C_inty-x_state(7);
    dx = C_intx-x_state(5);
    dist_to_overlap = sqrt(dy^2+dx^2);
    th_N = atan2(dy, dx); % Angle of normal force
end

function X = calc_instant_contact_vels(p, x_state, shape)
    X = x_state;
    disp('Calculating contact velocity')
    [th_N, ~, ~, ~] = calc_normal_angle(p, x_state, shape);
    VC_X = x_state(2); % x velocity of the cutter
    VC_Y = x_state(4); % y velocity of the cutter
    
    th_vc = atan2(VC_Y, VC_X); % Angle of the cutter's velocity
    th_proj = th_vc -th_N; % Projection angle between velocity and normal
    
    norm_vel = sqrt(VC_X^2+VC_Y^2)*cos(th_proj);
    X(6) = norm_vel*cos(th_N);
    X(8) = norm_vel*sin(th_N); 
end


%% Event Functions
function [eventVal, isterminal, direction] = freeBranchEvent(t,X,p)
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    
    % Position of center of cutter joint
    X_C = X(1);   Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    % translate the polyshape
    top_cutter = translate(p.top_shape, X_C, Y_C);
    bottom_cutter = translate(p.bottom_shape, X_C, Y_C);
%     t3 = 0:.01:2*pi;
%     branch = polyshape(p.r_branch.*cos(t3)+X_B, p.r_branch.*sin(t3)+Y_B);
    branch = translate(p.branch_shape, X_B, Y_B);
    overlap_shape_top = intersect(top_cutter, branch);
    overlap_shape_bottom = intersect(bottom_cutter, branch);
    
    a_top = area(overlap_shape_top);
    a_bot = area(overlap_shape_bottom);
    
%     disp('Area of overlap')
%     disp(area(overlap_shape));
    % ie 1 = contact with top cutter; ie 2 = contact with bottom cutter
    eventVal = [(a_top-a_bot)/(a_top+a_bot)-1, a_top-1e-8, a_bot-1e-8];
    isterminal = [1, 1, 1];     % stops the sim
    direction = [0, 1, 1];      % any direction
end
%
function [eventVal, isterminal, direction] = contactTopStickingEvent(t,X,p)
    % t: time, X: the state, p: parameters structure
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    % translate the polyshapes and find the overlap
    top_cutter = translate(p.top_shape, X_C, Y_C);
    bottom_cutter = translate(p.bottom_shape, X_C, Y_C);
    branch = translate(p.branch_shape, X_B, Y_B); 
    overlap_shape_top = intersect(top_cutter, branch);
    overlap_shape_bottom = intersect(bottom_cutter, branch);
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Time = ')
%     disp(t)
%     disp('Area of overlap =')
%     disp(area(overlap_shape_bottom)-5e-9);

    % 1: back to free
    % 2: to sliding
    % 3: to both
    eventVal = [area(overlap_shape_top)-5e-9, relVel_mag-.001, area(overlap_shape_bottom)-1e-8];   % when spring at equilibrium distance...
    isterminal = [1 1 1];     % stops the sim
    direction = [-1 1 1];      % direction
end

function [eventVal, isterminal, direction] = contactTopSlidingEvent(t,X,p)
    % t: time, X: the state, p: parameters structure
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    % translate the polyshape
    top_cutter = translate(p.top_shape, X_C, Y_C);
    bottom_cutter = translate(p.bottom_shape, X_C, Y_C);
    branch = translate(p.branch_shape, X_B, Y_B);
    overlap_shape_top = intersect(top_cutter, branch);
    overlap_shape_bottom = intersect(bottom_cutter, branch);
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
%     disp(t)
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Time = ')
%     disp(t)
%     disp('Area of overlap =')
%     disp(area(overlap_shape_bottom)-5e-9);

    % 1: back to free
    % 2: to sticking
    % 3: to both
    eventVal = [area(overlap_shape_top)-5e-9, relVel_mag-.001, area(overlap_shape_bottom)-1e-8];   % when spring at equilibrium distance...
    isterminal = [1, 1, 1];     % stops the sim
    direction = [-1, -1, 1];      % direction
end

function [eventVal, isterminal, direction] = contactBottomStickingEvent(t,X,p)
    % t: time, X: the state, p: parameters structure
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    % translate the polyshape
    bottom_cutter = translate(p.bottom_shape, X_C, Y_C);
    top_cutter = translate(p.top_shape, X_C, Y_C);
    branch = translate(p.branch_shape, X_B, Y_B);
    overlap_shape_bottom = intersect(bottom_cutter, branch);
    overlap_shape_top = intersect(top_cutter, branch);
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    % 1: to free
    % 2: to sliding
    % 3: to both
    eventVal = [area(overlap_shape_bottom)-5e-9, relVel_mag-.001, area(overlap_shape_top)-1e-8];   % when spring at equilibrium distance...
    isterminal = [1 1 1];     % stops the sim
    direction = [-1 1 1];      % any direction
end

function [eventVal, isterminal, direction] = contactBottomSlidingEvent(t,X,p)
    % t: time, X: the state, p: parameters structure
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    % translate the polyshape
    bottom_cutter = translate(p.bottom_shape, X_C, Y_C);
    top_cutter = translate(p.top_shape, X_C, Y_C);
    branch = translate(p.branch_shape, X_B, Y_B);
    overlap_shape_bottom = intersect(bottom_cutter, branch);
    overlap_shape_top = intersect(top_cutter, branch);
    
%     assert(area(overlap_shape_bottom) > 0);
%     assert(area(overlap_shape_top) > 0);
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    eventVal = [area(overlap_shape_bottom)-5e-9, relVel_mag-.001, area(overlap_shape_top)-1e-8];
    isterminal = [1 1 1];     % stops the sim
    direction = [-1 -1 1];      % any direction
end

function [eventVal, isterminal, direction] = bothcontactEvent(t,X,p)

    X_C = X(1);     Y_C = X(3); X_B = X(5); Y_B = X(7);
    
    bottom_cutter = translate(p.bottom_shape, X_C, Y_C);
    top_cutter = translate(p.top_shape, X_C, Y_C);
    branch = translate(p.branch_shape, X_B, Y_B);
    overlap_shape_bottom = intersect(bottom_cutter, branch);
    overlap_shape_top = intersect(top_cutter, branch);
    
%     disp('Event check! Area of bottom overlap: ')
%     disp(area(overlap_shape_bottom))
%     disp('Event chack! Area of top overlap: ')
%     disp(area(overlap_shape_top))
    
    eventVal = [area(overlap_shape_bottom)-6e-9, area(overlap_shape_top)-6e-9];
    isterminal = [1 1];
    direction = [-1 -1];

end
