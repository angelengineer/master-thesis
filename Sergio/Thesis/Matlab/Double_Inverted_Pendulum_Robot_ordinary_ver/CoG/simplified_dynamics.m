syms lt th_t(t) dth_t ddth_t th_w(t) dth_w ddth_w tau_w
M_t = M_b+M_f+M_m;

% Rotation matrices for theta_p and theta_a
Rt = simplify([cos(pi/2-th_t(t)) -sin(pi/2-th_t(t));
      sin(pi/2-th_t(t)) cos(pi/2-th_t(t))]);

% Linear velocity robot
v = [diff(th_w(t))*R_wo; 0];
%% --Wheels--

% Linear velocity
vw = v;

% Angular velocity
ww = v(1)/R_wo;

% Generalized velocity vector
vtw = [vw; ww];

% Unified inertia
Itw = 2*[M_w 0 0;
         0 M_w 0;
         0 0 I_wy];

% Kinetic energy
Kw = 1/2*vtw.'*Itw*vtw;
%% --Body--

% CoG coordinates w.r.t. P
Gt = Rt*[lt; 0];
% Angular velocity
wt = diff(th_t);

% Linear velocity
vt = vw+diff(Gt);

% Generalized velocity vector
vtb = [vt; wt];

% Unified inertia
I = [M_t 0 0;
     0 M_t 0;
     0 0 M_t*lt^2];

% Kinetic energy
Kt = simplify(1/2*vtb.'*I*vtb);

% Potential energy
Ut = M_t*g*Gt(2);
%% --Total Energy--
% Total U
U = Ut;

%Total K
K = simplify(expand(Kw+Kt));
%% --Lagrangian--
L = K - U;

% Generalized coordinates
q = [th_w(t), th_t(t)];
dq = [dth_w, dth_t];
ddq = [ddth_w, ddth_t];

% Preallocate vectors for dL_dq and dL_dt
dL_dq = sym(zeros(length(q), 1)); % Partial derivatives of L w.r.t. q
dL_dt = sym(zeros(length(q), 1)); % Time derivatives of partial derivatives of L w.r.t. dq

% Compute derivatives
for i = 1:length(q)
    % Partial derivative of L with respect to q(i)
    dL_dq(i) = collect(diff(L, q(i)),[diff(q,2), diff(q), q]);
    
    % Partial derivative of L with respect to dq(i)
    dL_ddq = diff(L, diff(q(i)));
    
    % Time derivative of the above partial derivative
    dL_dt(i) = collect(diff(dL_ddq, t),[diff(q,2), diff(q), q]);
end

% Generalized torques
tau = [tau_w; -tau_w];

% Euler-Lagrange equation
simp_EL_eq = dL_dt-dL_dq == tau;

% Find desired cog angle based on torque profile
syms ddth_w th

% t_cog = solve(subs(simp_EL_eq,[diff(th_t,2), diff(th_t), diff(th_w, 2), th_t],[0, 0, ddth_w, th]), [ddth_w, th], 'Real',true, 'ReturnConditions', true, MaxDegree=4);
% th = subs(t_cog.th(1),t_cog.parameters(1),0);
simp_EL_eq = subs(simp_EL_eq,[diff(th_t,2), diff(th_t), diff(th_w, 2), th_t],[0, 0, ddth_w, th]);
simp_EL_eq = subs(simp_EL_eq,[sin(th),cos(th)],[th,1]);

t_cog = solve(simp_EL_eq, [ddth_w, th]);
th = t_cog.th;

% matlabFunctionBlock('DIMR_ordinary/COG', th)