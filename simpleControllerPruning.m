function [Vx, Vy] = simpleControllerPruning(t, wrench, X)

    Kp = .5;
    Vy = Kp*wrench(2);
    Vx = X(2);
%     Vy = X(4);

end