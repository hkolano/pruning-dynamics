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
    t_end = .8;
    dt = 0.005;

    sol_set = {};
    
    p.top_shape = polyshape(p.top_x, p.top_y);
    p.bottom_shape = polyshape(p.bot_x, p.bot_y);
    t1 = 0:.02:2*pi;
    branch_xs = p.r_branch.*cos(t1);
    branch_ys = p.r_branch.*sin(t1);
    p.branch_shape = polyshape(branch_xs, branch_ys);
    
%     p.th_N = 0; % ODE will keep track of normal vector angle
    
    % States
    % 1: ball freefloating
    % 2: In contact with top blade/sticking
    % 3: In contact with top blade/sliding
    % 4: In contact with bottom blade/sticking
    % 5: In contact with bottom blade/sliding
    % 6: In contact with both blades
    p.state = 1;
    
    %% Function binding
    % Event functions
    free_event_fun = @(t,X)freeBranchEvent(t,X,p);  % When branch is freefloating
    top_sticking_event_fun = @(t,X)contactTopStickingEvent(t,X,p); % When in contact with top cutter
    bot_sticking_event_fun = @(t,X)contactBottomStickingEvent(t,X,p);
    top_sliding_event_fun = @(t,X)contactTopSlidingEvent(t,X,p);
    bot_sliding_event_fun = @(t,X)contactBottomSlidingEvent(t,X,p);

    % Simulation tolerances/ ODE Options
    optionsFree = odeset(...                    % When branch is freefloating
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', free_event_fun);
    optionsTopSticking = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', top_sticking_event_fun, 'MaxStep', .002);
    optionsTopSliding = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', top_sliding_event_fun, 'MaxStep', .002);
    optionsBottomSticking = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', bot_sticking_event_fun, 'MaxStep', .002);
    optionsBottomSliding = odeset(...                     % When in contact with top cutter -- TOLERANCE IS IMPORTANT 
        'RelTol', 1e-5, 'AbsTol', 1e-5, ...
        'Events', bot_sliding_event_fun, 'MaxStep', .001);

    % Bind dynamics function
    free_dyn_fun = @(t,X)freedyn(t,X,p);
    top_sticking_dyn_fun = @(t,X)stickingdyn(t,X,p,p.top_shape, 1);
    bottom_sticking_dyn_fun = @(t,X)stickingdyn(t,X,p,p.bottom_shape, -1);
    top_sliding_dyn_fun = @(t,X)slidingdyn(t,X,p,p.top_shape, 1);
    bottom_sliding_dyn_fun = @(t,X)slidingdyn(t,X,p,p.bottom_shape, -1);
    both_dyn_fun = @(t,X)bothhit(t,X,p)

    %% Iterate over all time
    while t_start < t_end
        % Simulate the dynamics over a time interval
        if p.state == 1 % (branch not in contact)
            disp('Starting Free dynamics')
            sol = ode45(free_dyn_fun, [t_start,t_end], X0, optionsFree);
            disp('Contact at t = ')
            disp(sol.x(end))
            if sol.ie == 1 % If trigger top contact in event
                p.state = 3; % Change to top sliding dynamics
            elseif sol.ie == 2
                p.state = 5; % Change to bottom sliding dynamics
            end
        elseif p.state == 2 % p.state == 2 (sticking to top blade)
            disp('Contact with top -- sticking')
%             X0 = calc_instant_contact_vels(p, X0, p.top_shape);
            sol = ode45(top_sticking_dyn_fun, [t_start,t_end], X0, optionsTopSticking);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1 % to Free
                p.state = 1; 
            elseif sol.ie == 2 % to Top/Sliding
                p.state = 3; 
            end
        elseif p.state == 3 % p.state == 2 (sliding on top blade)
            disp('Contact with top -- sliding')
            X0 = calc_instant_contact_vels(p, X0, p.top_shape);
            sol = ode45(top_sliding_dyn_fun, [t_start,t_end], X0, optionsTopSliding);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1
                p.state = 1; % to Free
            elseif sol.ie == 2
                p.state = 2; % to Top/Sticking
            elseif sol.ie == 3
                p.state = 6;
            end
        elseif p.state == 4 %(sticking to bottom blade)
            disp('Contact with bottom -- sticking')
%             X0 = calc_instant_contact_vels(p, X0, p.bottom_shape); 
            sol = ode45(bottom_sticking_dyn_fun, [t_start,t_end], X0, optionsBottomSticking);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1
                p.state = 1; % to Free
            elseif sol.ie == 2
                p.state = 5; % to Bottom/Sliding
            end
        elseif p.state == 5 %(sliding on bottom blade)
            disp('Contact with bottom -- sliding')
            X0 = calc_instant_contact_vels(p, X0, p.bottom_shape);
            sol = ode45(bottom_sliding_dyn_fun, [t_start,t_end], X0, optionsBottomSliding);
            disp('Change at t = ')
            disp(sol.x(end))
            if sol.ie == 1
                p.state = 1; % to Free
            elseif sol.ie == 2
                p.state = 4; % to Bottom/sticking
            elseif sol.ie == 3
                p.state = 6;
            end
        else % p.state = 6
            disp('Contact with both!')
            break
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

function dX = stickingdyn(t,X,p,shape, dir)
%     disp('Time = ')
%     disp(t)
    
    X_C = X(1);  Y_C = X(3); X_B = X(5); Y_B = X(7);
       
    % Restoring forces to the branch
    F_Kx = -p.kx*X_B;
    F_Ky = -p.ky*Y_B;
    F_K = sqrt(F_Ky^2+F_Kx^2);
    
    % Angle of restoring force (from +x)
    th_Fk = atan2(F_Ky, F_Kx);
    
    % Calculate the current Normal angle th_N
    [th_N, dist_to_overlap] = calc_normal_angle(p, X, shape);

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

    dX = zeros(length(X),1);
    dX(1) = X(2);  dX(3) = X(4);  dX(5) = X(6);  dX(7) = X(8);
   
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
   
   dX(6) = (F_Kx+F_Nx+F_fx)/p.m_branch;
   dX(8) = (F_Ky+F_Ny+F_fy)/p.m_branch;
   
end

function dX = slidingdyn(t,X,p,shape, dir)
    
    X_C = X(1);  Y_C = X(3); X_B = X(5); Y_B = X(7);
       
    % Restoring forces to the branch
    F_Kx = -p.kx*X(5);%-p.b*X(6);
    F_Ky = -p.ky*X(7);%-p.b*X(8);
    F_K = sqrt(F_Ky^2+F_Kx^2);
    
%     th_Fk = atan(F_Ky/F_Kx); % Angle of net restoring force (from horiz)
    th_Fk = atan2(F_Ky, F_Kx);
    
    % Calculate the current Normal angle th_N
    [th_N, dist_to_overlap] = calc_normal_angle(p, X, shape);

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
   
   dX(6) = (F_Kx+F_Nx+F_fx)/p.m_branch;
   dX(8) = (F_Ky+F_Ny+F_fy)/p.m_branch;
   
end

function [th_N, dist_to_overlap] = calc_normal_angle(p, x_state, shape)
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
    [th_N, ~] = calc_normal_angle(p, x_state, shape);
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
    overlap_shape_bot = intersect(bottom_cutter, branch);
    
%     disp('Area of overlap')
%     disp(area(overlap_shape));
    % ie 1 = contact with top cutter; ie 2 = contact with bottom cutter
    eventVal = [area(overlap_shape_top)-1e-8, area(overlap_shape_bot)-1e-8];
    isterminal = [1, 1];     % stops the sim
    direction = [1, 1];      % any direction
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
    branch = translate(p.branch_shape, X_B, Y_B); 
    overlap_shape = intersect(top_cutter, branch);
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    % 1: back to free
    % 2: to sliding
    eventVal = [area(overlap_shape)-5e-9, relVel_mag-.001];   % when spring at equilibrium distance...
    isterminal = [1 1];     % stops the sim
    direction = [-1 1];      % direction
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

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    % 1: back to free
    % 2: to sticking
    % 3: to both
    eventVal = [area(overlap_shape_top)-5e-9, relVel_mag-.001, area(overlap_shape_bottom)-5e-9];   % when spring at equilibrium distance...
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
    branch = translate(p.branch_shape, X_B, Y_B);
    overlap_shape = intersect(bottom_cutter, branch);
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    eventVal = [area(overlap_shape)-5e-9, relVel_mag-.001];   % when spring at equilibrium distance...
    isterminal = [1 1];     % stops the sim
    direction = [-1 1];      % any direction
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
    
    relVelX = X(6)-X(2);
    relVelY = X(8)-X(4);
%     th_relV_ang = atan2(relVelY, relVelX);
    relVel_mag = sqrt(relVelY^2 + relVelX^2);

%     disp('Event Check! Area of overlap')
%     disp(area(overlap_shape));

    eventVal = [area(overlap_shape_bottom)-5e-9, relVel_mag-.001, area(overlap_shape_top)-5e-9];   % when spring at equilibrium distance...
    isterminal = [1 1 1];     % stops the sim
    direction = [-1 -1 1];      % any direction
end

