function [F_Kx, F_Ky, F_K, th_Fk] = getRestoringForces(p, x_state)
    % Inputs:   p (parameter structure, need p.kx and p.ky)
    %           x_state (current state of cutter and branch)
    % Outputs:  F_Kx, F_Ky (restoring force of branch in X and Y directions)
    %           F_K (magnitude of restoring force)
    %           th_Fk (angle of restoring force from +x, from 0 to pi (or
    %           -pi)
    
%     X_C = X(1);  Y_C = X(3); X_B = X(5); Y_B = X(7);
    F_Kx = -p.kx*x_state(5);
    F_Ky = -p.ky*x_state(7);
    F_K = sqrt(F_Ky^2+F_Kx^2);
    th_Fk = atan2(F_Ky, F_Kx);

end