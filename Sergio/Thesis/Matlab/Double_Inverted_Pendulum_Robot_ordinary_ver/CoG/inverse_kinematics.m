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
syms dz z th_t lt tau mass_l

% Define the rotation matrix for the first joint (theta1)
Rp = simplify([cos(pi/2 - theta1), -sin(pi/2 - theta1);
               sin(pi/2 - theta1),  cos(pi/2 - theta1)]);

% Define the rotation matrix for the third joint (theta3)
Ra = [cos(-theta3), -sin(-theta3);
      sin(-theta3),  cos(-theta3)];

% Compute the center of mass positions for each link
Gb = Rp * [L_bg; 0];  % Center of mass of the base
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
eqn1 = simplify(CoG(1)) == 0;
eqn2 = H_BT(3,4) == z;
eqns = [eqn1; eqn2];

% Define unified diff-ik equations (CoG X coordinate and Z fork height)
syms q1(t) q2(t) q3(t) dtheta1 ddelta2
deqn1 = diff(subs(CoG(1),[theta1, delta2],[q1(t), q2(t)])) == 0;
deqn2 = diff(subs(H_BT(3,4),[theta1, delta2],[q1(t), q2(t)])) == dz;
deqns = [deqn1; deqn2];
deqns = subs(deqns,[diff(q1) q1 diff(q2) q2],[dtheta1 theta1 ddelta2 delta2]);

full_eqns = [eqns; deqns];
full_eqns = subs(full_eqns,[sin(theta1),cos(theta1)],[theta1,1]);

% ik = solve([eqns; deqns], [ddelta2 delta2 dtheta1 theta1],'ReturnConditions', true, MaxDegree=4);
% ik = solve([eqns; deqns], [ddelta2 delta2 dtheta1 theta1]);
ik = solve(full_eqns, [ddelta2 delta2 dtheta1 theta1]);

ik_theta1 = ik.theta1;
ik_delta2 = ik.delta2;
ik_dtheta1 = ik.dtheta1;
ik_ddelta2 = ik.ddelta2;
ik_CoG = subs(CoG,[theta1, delta2],[ik_theta1(1), ik_delta2(1)]);
ik_lt = sqrt(ik_CoG.'*ik_CoG);

% ik_theta1 = subs(ik.theta1(7),ik.parameters(1),0);
% ik_delta2 = subs(ik.delta2(7),ik.parameters(1),0);
% ik_dtheta1 = subs(ik.dtheta1(7),ik.parameters(1),0);
% ik_ddelta2 = subs(ik.ddelta2(7),ik.parameters(1),0);
% ik_CoG = subs(CoG,[theta1, delta2],[ik_theta1, ik_delta2]);
% ik_lt = sqrt(ik_CoG.'*ik_CoG);

matlabFunctionBlock('DIMR_ordinary/IK_simp', ik_theta1, ik_delta2, ik_dtheta1, ik_ddelta2, ik_lt)

% matlabFunctionBlock('DIMR_ordinary/IK', ik_theta1, ik_delta2, ik_dtheta1, ik_ddelta2, ik_lt)

simplified_dynamics;

% Define inverse cog equations
%cog_eq1 = tan(th_t) == CoG(1)/CoG(2);
cog_eq1 = tan(th) == CoG(1)/CoG(2);
cog_eq2 = lt^2 == CoG.'*CoG;
cog_eqs = [cog_eq1; cog_eq2];
cog_eqs = subs(cog_eqs,[sin(theta1),cos(theta1),tan(th)],[theta1,1,th]);

% syms q1(t) q2(t) h1(t) dtheta1 ddelta2
% dcog_eq1 = diff(subs(CoG(1)/CoG(2),[theta1, delta2],[q1(t), q2(t)])) == diff(subs(tan(th),th,h1(t)));
% dcog_eq2 = diff(subs(CoG.'*CoG,[theta1, delta2],[q1(t), q2(t)])) == 0;
% dcog_eqs = [dcog_eq1; dcog_eq2];
% dcog_eqs = subs(dcog_eqs,[diff(q1) q1 diff(q2) q2 diff(h1) h1],[dtheta1 theta1 ddelta2 delta2 0 th]);
% 
% full_cog_eqs = [cog_eqs; dcog_eqs];
% full_cog_eqs = subs(full_cog_eqs,[sin(theta1),cos(theta1),sin(th),tan(th),cos(th)],[theta1,1,th,th,1]);

% inv_cog = solve(full_cog_eqs,[ddelta2 delta2 dtheta1 theta1]);
% inv_cog_theta1 = inv_cog.theta1;
% inv_cog_delta2 = inv_cog.delta2;
% inv_cog_dtheta1 = inv_cog.dtheta1;
% inv_cog_ddelta2 = inv_cog.ddelta2;

inv_cog = solve(cog_eqs,[delta2 theta1]);
inv_cog_theta1 = inv_cog.theta1(1);
inv_cog_delta2 = inv_cog.delta2(1);
% inv_cog_theta1 = subs(inv_cog.theta1(3),inv_cog.parameters(1),0);
% inv_cog_delta2 = subs(inv_cog.delta2(3),inv_cog.parameters(1),0);
inv_cog_dtheta1 = 0;
inv_cog_ddelta2 = 0;

matlabFunctionBlock('DIMR_ordinary/simp_model_simp', inv_cog_theta1, inv_cog_delta2, inv_cog_dtheta1, inv_cog_ddelta2)

function H=HT(type, alpha, a, theta, d, offset)
    if strcmp(type, "r")        
        theta = theta+offset;    
    elseif strcmp(type, "p")
        d = d+offset;
    end
    H = [rotx(alpha),zeros(3,1);[0 0 0 1]]*[eye(3),[a; 0; 0];[0 0 0 1]]*[rotz(theta),zeros(3,1);[0 0 0 1]]*[eye(3),[0; 0; d];[0 0 0 1]];
end
