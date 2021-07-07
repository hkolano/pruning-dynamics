X = [0 0 0 0 0 0 0 0];
C = [.033 0 .0035 0];
F = [0 0 1 0 0 0 0 0];

wrench = getForceTorqueMeasurement(p, X, F, C)