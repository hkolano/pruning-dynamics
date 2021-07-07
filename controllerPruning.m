function [Vx, Vy] = controllerPruning(t, wrench, X)

    Kp = 1;
    Vy = -Kp*wrench(2);
    Vx = X(2);
    Vy = X(4);

end