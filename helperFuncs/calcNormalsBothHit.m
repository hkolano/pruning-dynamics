function [centpoints, th_T, th_B, dist_to_overlap_top, dist_to_overlap_bottom] = calcNormalsBothHit(p,X)
    % Translate polyshapes
    top_cutter = translate(p.top_shape, X(1), X(3));
    bottom_cutter = translate(p.bottom_shape, X(1), X(3));
    branch = translate(p.branch_shape, X(5), X(7));
    
    % Get intersects between cutter and branch
    poly_int_top = intersect(top_cutter, branch);
    poly_int_bottom = intersect(bottom_cutter, branch);  
    
    assert(area(poly_int_top) > 0)
    assert(area(poly_int_bottom) > 0)
    
    [C_intx_top, C_inty_top] = centroid(poly_int_top);
    [C_intx_bot, C_inty_bot] = centroid(poly_int_bottom); 
    
    % Find angle of the normal/top
    dy_top = C_inty_top-X(7);
    dx_top = C_intx_top-X(5);
    th_T = atan2(dy_top,dx_top); % Angle of normal force from top
    dist_to_overlap_top = sqrt(dy_top^2+dx_top^2);
       
    % Find angle of the normal/bottom
    dy_bottom = C_inty_bot-X(7);
    dx_bottom = C_intx_bot-X(5);
    th_B = atan2(dy_bottom, dx_bottom);
    dist_to_overlap_bottom = sqrt(dy_bottom^2+dx_bottom^2);
    
    % Group centers of the intersections
    centpoints = [C_intx_top, C_intx_bot, C_inty_top, C_inty_bot];
end