% MATLAB (Symbolic) - derivar Lagrangiano cart-pole (pivot inertia)
syms M m L g b F real
syms x th xdot thdot xdd thdd real

l = L/2;
I_pivot = (1/3)*m*L^2;

% Variables (symbolic time not necessary if do algebra)
T = 1/2*(M + m)*xdot^2 + 1/2*I_pivot*thdot^2;
V = m*g*l*(1 - cos(th));
Lagr = T - V;
R = 1/2*b*xdot^2;

% Compute partial derivatives for EL:
dL_dxdot = diff(Lagr, xdot);
d_dt_dL_dxdot = diff(dL_dxdot, xdot)*xdd + diff(dL_dxdot, thdot)*thdd; % chain rule simplified
dL_dx = 0; % no x in Lagr for this model
dR_dxdot = diff(R, xdot);

dL_dthdot = diff(Lagr, thdot);
d_dt_dL_dthdot = diff(dL_dthdot, xdot)*xdd + diff(dL_dthdot, thdot)*thdd;
dL_dth = diff(Lagr, th);

% Form equations
eq1 = simplify(d_dt_dL_dxdot - dL_dx + dR_dxdot - F);
eq2 = simplify(d_dt_dL_dthdot - dL_dth);

% Display (you can then solve for xdd, thdd)
eq1 = collect(eq1, [xdd, thdd])
eq2 = collect(eq2, [xdd, thdd])

% Optionally generate numeric functions:
% matlabFunction(eq1, eq2, 'Vars', {M,m,L,g,b,F, xdot, thdot, xdd, thdd, th});
