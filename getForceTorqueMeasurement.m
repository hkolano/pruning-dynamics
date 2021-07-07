function wrench = getForceTorqueMeasurement(p, x_state, Forces, ContactPoints)
    % Extract relevant values
    ContactPointCells = num2cell(ContactPoints);
    ForcesCells = num2cell(Forces);
    [CPtopX, CPbottomX, CPtopY, CPbottomY] = ContactPointCells{:};
    [FNtopX, FNbottomX, FNtopY, FNbottomY, FftopX, FfbottomX, FftopY, FfbottomY] = ForcesCells{:};
    
    % Determine wrench in contact frames
    wrench_in_t = [0 0 0 FNtopX+FftopX FNtopY+FftopY 0]';
    wrench_in_b = [0 0 0 FNbottomX+FfbottomX FNbottomY+FfbottomY 0]';
    
    % Determine transform to contact frames from joint 
    T_c_top = [1 0 0 CPtopX-x_state(1);
                0 1 0 CPtopY-x_state(3);
                0 0 1 0; 
                0 0 0 1];
    T_c_bottom = [1 0 0 CPbottomX-x_state(1);
                0 1 0 CPbottomY-x_state(3);
                0 0 1 0; 
                0 0 0 1];
            
    T_ee_top = p.T_ee_cutter*T_c_top;
    T_ee_bottom = p.T_ee_cutter*T_c_bottom;
    
    top_wrench_in_ee = Adjoint(TransInv(T_ee_top))'*wrench_in_t;
    bottom_wrench_in_ee = Adjoint(TransInv(T_ee_bottom))'*wrench_in_b;
    
    wrench = top_wrench_in_ee + bottom_wrench_in_ee
end