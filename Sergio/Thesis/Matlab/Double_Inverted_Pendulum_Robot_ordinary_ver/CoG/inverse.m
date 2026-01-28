%% Kinematic chain
threshold = 1e-10;

% Define symbolic variables for prismatic and revolute joint displacements
syms theta1 delta2 theta3;

% Define the base transformation matrix (homogeneous transformation)
base = [1, 0, 0, 0;
        0, 0, 1, 0;
        0, -1, 0, R_wo;
        0, 0, 0, 1];

% Define the tool transformation matrix (homogeneous transformation)
tool = [1, 0, 0, L_fg;
        0, 0, -1, 0;
        0, 1, 0, 0;
        0, 0, 0, 1];

H_01 = HT("r", 0, 0, theta1, 0, 0);
H_01 = mapSymType(H_01, 'rational', @(x) piecewise(abs(x) <= threshold, 0, x));
H_12 = HT("p", pi/2, 0, 0, delta2, L_mg);
H_12 = mapSymType(H_12, 'rational', @(x) piecewise(abs(x) <= threshold, 0, x));
H_23 = HT("r", -pi/2, 0, theta3, 0, 0);
H_23 = mapSymType(H_23, 'rational', @(x) piecewise(abs(x) <= threshold, 0, x));

% Compute the transformation matrices from the base to each joint
H_B1 = simplify(base * H_01);  % Base to Joint 1
H_B2 = simplify(H_B1 * H_12);  % Base to Joint 2
H_B3 = simplify(H_B2 * H_23);  % Base to Joint 3

% Compute the final transformation matrix from the base to the tool
H_BT = simplify(H_B3 * tool);
%% CoG
% Define symbolic variables for link lengths, masses, and other parameters
syms dz z th lt tau mass_l

% Define the rotation matrix for the first joint (theta1)
Rp = simplify([cos(pi/2 - theta1), -sin(pi/2 - theta1);
               sin(pi/2 - theta1),  cos(pi/2 - theta1)]);

% Define the rotation matrix for the third joint (theta3)
Ra = [cos(-theta3), -sin(-theta3);
      sin(-theta3),  cos(-theta3)];

% Compute the center of mass positions for each link
% Gb = Rp * [L_bmg; 0];  % Center of mass of the base
Gb = Rp * [C_bz; C_bx];  % Center of mass of the base
Gm = Rp * [L_mg + delta2; 0];  % Center of mass of the middle link

% Compute the rotated position of the arm's center of mass
GaR = simplify(expand(Rp * ([L_mg + delta2; 0] + Ra * [0; -L_fg])));

% Compute the rotated position of the load's center of mass
Gl = simplify(expand(Rp * ([L_mg + delta2; 0] + Ra * [0; -tau/(mass_l*g)])));

% Compute the overall center of gravity of the robot
CoG = simplify((M_b * Gb + M_m * Gm + M_f * GaR + mass_l * Gl) / (M_b + M_m + M_f + mass_l));

% Assume the fork paralell to the floor
CoG = subs(CoG,theta1+theta3,0);
H_BT = subs(H_BT,theta1+theta3,0);

% Define unified ik equations (CoG X coordinate and Z fork height)
eqn1 = simplify(CoG(1)) == sin(th)*lt;
% eqn1 = simplify(CoG(1)/CoG(2)) == tan(th);
eqn2 = H_BT(3,4) == z;
eqns = [eqn1; eqn2];

% Define unified diff-ik equations (CoG X coordinate and Z fork height)
syms q1(t) q2(t) q3(t) dtheta1 ddelta2
deqn1 = dtheta1 == 0;
% deqn1 = diff(subs(CoG(1),[theta1, delta2],[q1(t), q2(t)])) == dtheta1;
% deqn1 = diff(subs(CoG(1),[theta1, delta2],[q1(t), q2(t)])) == 0;
deqn2 = diff(subs(H_BT(3,4),[theta1, delta2],[q1(t), q2(t)])) == dz;
deqns = [deqn1; deqn2];
deqns = subs(deqns,[diff(q1) q1 diff(q2) q2],[dtheta1 theta1 ddelta2 delta2]);

ik_eqns = [eqns; deqns];
ik_eqns = subs(ik_eqns,[sin(theta1),cos(theta1),sin(th)],[theta1,1,th]);
% ik_eqns = subs(ik_eqns,[sin(theta1),cos(theta1),tan(th)],[theta1,1,th]);
% ik_eqns = vpa(ik_eqns,6);

%% Simplified dynamics
syms th_t(t) dth_t ddth_t th_w(t) dth_w ddth_w tau_w
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
% Gt = CoG;
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
% I = [M_t 0 0;
%      0 M_t 0;
%      0 0 M_t*(CoG.'*CoG)];

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

simp_EL_eq = subs(simp_EL_eq,[diff(th_t,2), diff(th_t), diff(th_w, 2), th_t],[0, 0, ddth_w, th]);
simp_EL_eq = subs(simp_EL_eq,[sin(th),cos(th)],[th,1]);
% simp_EL_eq = subs(simp_EL_eq,[sin(theta1),cos(theta1),tan(th),sin(th),cos(th)],[theta1,1,th,th,1]);
% simp_EL_eq = vpa(simp_EL_eq,6);


% Define inverse cog equations
cog_eqs = lt^2 == CoG.'*CoG;
cog_eqs = subs(cog_eqs,[sin(theta1),cos(theta1),tan(th)],[theta1,1,th]);
% cog_eqs = vpa(cog_eqs,6);

% full_eqns = [ik_eqns; simp_EL_eq];
full_eqns = [ik_eqns; simp_EL_eq; cog_eqs];
% full_eqns = vpa(full_eqns,6);

inv_s = solve(full_eqns, [ddth_w, th, lt, ddelta2, delta2, dtheta1, theta1]);

sol_theta1 = inv_s.theta1;
% sol_theta1 = expand(sol_theta1);
% sol_theta1 = simplify(sol_theta1);

sol_delta2 = inv_s.delta2(1);
% sol_delta2 = expand(sol_delta2);
% sol_delta2 = simplify(sol_delta2);

sol_dtheta1 = inv_s.dtheta1(1);
% sol_dtheta1 = expand(sol_dtheta1);
% sol_dtheta1 = simplify(sol_dtheta1);

sol_ddelta2 = inv_s.ddelta2(1);
% sol_ddelta2 = expand(sol_ddelta2);
% sol_ddelta2 = simplify(sol_ddelta2);

matlabFunctionBlock('DIMR_ordinary/Solution', sol_theta1, sol_delta2, sol_dtheta1, sol_ddelta2)

function H=HT(type, alpha, a, theta, d, offset)
    if strcmp(type, "r")        
        theta = theta+offset;    
    elseif strcmp(type, "p")
        d = d+offset;
    end
    H = [rotx(alpha),zeros(3,1);[0 0 0 1]]*[eye(3),[a; 0; 0];[0 0 0 1]]*[rotz(theta),zeros(3,1);[0 0 0 1]]*[eye(3),[0; 0; d];[0 0 0 1]];
end
