function [F_NTx, F_NTy, F_NBx, F_NBy] = getNormalForcesBothBlades(th_T, th_B, F_Kx, F_Ky)
    th_Tstar = th_T-pi;
    th_Bstar = pi+th_B;
%     F_NB = (F_Kx - F_Ky)/(cos(th_B+pi/2) - sin(th_B+pi/2));
%     F_NT = (F_Ky*cos(th_B+pi/2) - F_Kx*sin(th_B+pi/2))/(cos(th_T - pi/2)*(cos(th_B + pi/2) - sin(th_B + pi/2)));
    F_NT = (-F_Ky*cos(th_Bstar) + F_Kx*sin(th_Bstar))/(sin(th_Tstar)*cos(th_Bstar)-cos(th_Tstar)*sin(th_Bstar));
    F_NB = (-F_Kx-F_NT*cos(th_Tstar))/cos(th_Bstar);
 
    F_NTx = F_NT*cos(th_Tstar);
    F_NTy = F_NT*sin(th_Tstar);
    
    F_NBx = F_NB*cos(th_Bstar);
    F_NBy = F_NB*sin(th_Bstar);
end