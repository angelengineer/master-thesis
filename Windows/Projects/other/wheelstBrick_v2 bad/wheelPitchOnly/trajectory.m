%% References
x = [0;0;0;0];
u = [0;0];

ref_0 = [x;u];
ref = repmat(x, N-1, 1);
ref_e = x;

% Limits
ubx = [100;pi/4;3;3];
lbx = -ubx;

upu = [20];
lbu = -upu;
