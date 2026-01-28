%% Wheeled Robot Lagrangian Dynamics Calculator
% This script computes the equations of motion for a wheeled robot with
% lift mechanism and fork using the Lagrangian formulation
% Modified to use x (position) instead of theta_w (wheel angle)

clear; clc;

%% Define Symbolic Variables
% Generalized coordinates
syms x theta_p d_m theta_a real
% Generalized velocities
syms dx dtheta_p dd_m dtheta_a real
% Generalized accelerations
syms ddx ddtheta_p ddd_m ddtheta_a real

% Physical parameters
syms r_w g real  % wheel radius, gravity

% Mass parameters
syms m_w m_b m_m m_a m_f real  % masses of wheel, body, lift, arm, fork

% Inertia parameters
syms I_w I_b I_m I_a I_f real  % moments of inertia

% Geometric parameters (CoG locations)
syms l_bx l_bz real  % body CoG
syms l_mx l_mz real  % lift assembly CoG
syms l_ax l_az real  % arm motor CoG
syms l_fx real       % fork length (distance from arm motor to fork CoG)

%% Define Generalized Coordinates Vector
q = [x; theta_p; d_m; theta_a];
dq = [dx; dtheta_p; dd_m; dtheta_a];
ddq = [ddx; ddtheta_p; ddd_m; ddtheta_a];

%% 1. WHEEL SUBSYSTEM
% Kinetic energy (note: dtheta_w = dx/r_w)
T_w = 0.5 * m_w * dx^2 + 0.5 * I_w * (dx/r_w)^2;

% Potential energy
U_w = 0;

%% 2. MAIN BODY SUBSYSTEM
% Position
x_b = x + l_bx*cos(theta_p) + l_bz*sin(theta_p);
z_b = r_w - l_bx*sin(theta_p) + l_bz*cos(theta_p);

% Velocity
dx_b = dx - l_bx*dtheta_p*sin(theta_p) + l_bz*dtheta_p*cos(theta_p);
dz_b = -l_bx*dtheta_p*cos(theta_p) - l_bz*dtheta_p*sin(theta_p);

% Kinetic energy
T_b = 0.5 * m_b * (dx_b^2 + dz_b^2) + 0.5 * I_b * dtheta_p^2;

% Potential energy
U_b = m_b * g * (-l_bx*sin(theta_p) + l_bz*cos(theta_p));

%% 3. LIFT SUBSYSTEM
% Position
x_m = x + l_mx*cos(theta_p) + l_mz*sin(theta_p);
z_m = r_w - l_mx*sin(theta_p) + l_mz*cos(theta_p);

% Velocity
dx_m = dx - l_mx*dtheta_p*sin(theta_p) + l_mz*dtheta_p*cos(theta_p);
dz_m = -l_mx*dtheta_p*cos(theta_p) - l_mz*dtheta_p*sin(theta_p);

% Kinetic energy
T_m = 0.5 * m_m * (dx_m^2 + dz_m^2) + 0.5 * I_m * dtheta_p^2;

% Potential energy
U_m = m_m * g * (-l_mx*sin(theta_p) + l_mz*cos(theta_p));

%% 4. FORK MOTOR ARM SUBSYSTEM
% Position
x_a = x + l_ax*cos(theta_p) + (l_az + d_m)*sin(theta_p);
z_a = r_w - l_ax*sin(theta_p) + (l_az + d_m)*cos(theta_p);

% Velocity
dx_a = dx - l_ax*dtheta_p*sin(theta_p) + ...
       (l_az + d_m)*dtheta_p*cos(theta_p) + dd_m*sin(theta_p);
dz_a = -l_ax*dtheta_p*cos(theta_p) - ...
       (l_az + d_m)*dtheta_p*sin(theta_p) + dd_m*cos(theta_p);

% Kinetic energy
T_a = 0.5 * m_a * (dx_a^2 + dz_a^2) + 0.5 * I_a * dtheta_p^2;

% Potential energy
U_a = m_a * g * (-l_ax*sin(theta_p) + (l_az + d_m)*cos(theta_p));

%% 5. FORK TOOL SUBSYSTEM
% Position (note: uses l_ax for x-position, l_fx for fork length)
x_f = x + l_ax*cos(theta_p) + (l_az + d_m)*sin(theta_p) + ...
      l_fx * cos(theta_p + theta_a);
z_f = r_w - l_ax*sin(theta_p) + (l_az + d_m)*cos(theta_p) - ...
      l_fx * sin(theta_p + theta_a);

% Velocity
dx_f = dx - l_ax*dtheta_p*sin(theta_p) + ...
       (l_az + d_m)*dtheta_p*cos(theta_p) + dd_m*sin(theta_p) - ...
       l_fx * sin(theta_p + theta_a)*(dtheta_p + dtheta_a);
dz_f = -l_ax*dtheta_p*cos(theta_p) - ...
       (l_az + d_m)*dtheta_p*sin(theta_p) + dd_m*cos(theta_p) - ...
       l_fx * cos(theta_p + theta_a)*(dtheta_p + dtheta_a);

% Kinetic energy
T_f = 0.5 * m_f * (dx_f^2 + dz_f^2) + 0.5 * I_f * (dtheta_p + dtheta_a)^2;

% Potential energy
U_f = m_f * g * (-l_ax*sin(theta_p) + (l_az + d_m)*cos(theta_p) - ...
                 l_fx * sin(theta_p + theta_a));

%% TOTAL ENERGY
% Note: Multiplying wheel energy by 2 (two wheels)
T_total = 2*T_w + T_b + T_m + T_a + T_f;
U_total = 2*U_w + U_b + U_m + U_a + U_f;

% Lagrangian
L = T_total - U_total;

fprintf('Computing Lagrangian dynamics...\n');

%% COMPUTE EQUATIONS OF MOTION
% M(q)*ddq + C(q,dq)*dq + G(q) = tau

% Initialize matrices
n = length(q);
M = sym(zeros(n, n));
C = sym(zeros(n, n));
G = sym(zeros(n, 1));

fprintf('Computing Mass Matrix M...\n');
% Mass matrix M
for i = 1:n
    for j = 1:n
        M(i,j) = diff(diff(L, dq(j)), dq(i));
    end
end

fprintf('Computing Coriolis and Centrifugal terms C...\n');
% Coriolis matrix C using Christoffel symbols
for k = 1:n
    for j = 1:n
        c_kj = 0;
        for i = 1:n
            c_kj = c_kj + 0.5 * (diff(M(k,j), q(i)) + ...
                                  diff(M(k,i), q(j)) - ...
                                  diff(M(i,j), q(k))) * dq(i);
        end
        C(k,j) = c_kj;
    end
end

fprintf('Computing Gravity Vector G...\n');
% Gravity vector G (from potential energy only)
for i = 1:n
    G(i) = diff(U_total, q(i));
end

% Simplify
fprintf('Simplifying expressions (this may take a while)...\n');
M = simplify(M);
C = simplify(C);
G = simplify(G);

%% DISPLAY RESULTS
fprintf('\n=== DYNAMICS EQUATIONS ===\n');
fprintf('M(q)*ddq + C(q,dq)*dq + G(q) = tau\n\n');

fprintf('Mass Matrix M:\n');
disp(M);

fprintf('\nCoriolis Matrix C:\n');
disp(C);

fprintf('\nGravity Vector G:\n');
disp(G);

%% GENERATE MATLAB FUNCTIONS
fprintf('\nGenerating MATLAB functions...\n');

% Generate function for mass matrix
matlabFunction(M, 'File', 'mass_matrix', ...
    'Vars', {x, theta_p, d_m, theta_a, ...
             m_w, m_b, m_m, m_a, m_f, ...
             I_w, I_b, I_m, I_a, I_f, ...
             r_w, l_bx, l_bz, l_mx, l_mz, l_ax, l_az, l_fx}, ...
    'Optimize', false);

% Generate function for Coriolis matrix
matlabFunction(C, 'File', 'coriolis_matrix', ...
    'Vars', {x, theta_p, d_m, theta_a, ...
             dx, dtheta_p, dd_m, dtheta_a, ...
             m_w, m_b, m_m, m_a, m_f, ...
             I_w, I_b, I_m, I_a, I_f, ...
             r_w, l_bx, l_bz, l_mx, l_mz, l_ax, l_az, l_fx}, ...
    'Optimize', false);

% Generate function for gravity vector
matlabFunction(G, 'File', 'gravity_vector', ...
    'Vars', {x, theta_p, d_m, theta_a, ...
             m_w, m_b, m_m, m_a, m_f, ...
             g, r_w, l_bx, l_bz, l_mx, l_mz, l_ax, l_az, l_fx}, ...
    'Optimize', false);

fprintf('Functions generated: mass_matrix.m, coriolis_matrix.m, gravity_vector.m\n');

%% EXAMPLE: NUMERICAL EVALUATION
fprintf('\n=== EXAMPLE NUMERICAL EVALUATION ===\n');

% Define numerical parameters (example values - adjust as needed)
params = struct();
params.m_w = 2.0;    % kg
params.m_b = 10.0;   % kg
params.m_m = 3.0;    % kg
params.m_a = 1.5;    % kg
params.m_f = 2.0;    % kg

params.I_w = 0.01;   % kg*m^2
params.I_b = 0.5;    % kg*m^2
params.I_m = 0.1;    % kg*m^2
params.I_a = 0.05;   % kg*m^2
params.I_f = 0.08;   % kg*m^2

params.r_w = 0.15;   % m
params.l_bx = 0.0;   % m
params.l_bz = 0.2;   % m
params.l_mx = 0.0;   % m
params.l_mz = 0.4;   % m
params.l_ax = 0.0;   % m
params.l_az = 0.5;   % m
params.l_fx = 0.3;   % m (fork length)

params.g = 9.81;     % m/s^2

% State (example configuration)
q_num = [0.075; 0.1; 0.2; -0.3];       % [x, theta_p, d_m, theta_a]
dq_num = [0.15; 0.5; 0.1; 0.2];        % velocities [dx, dtheta_p, dd_m, dtheta_a]

% Use the generated functions to evaluate numerically
M_num = mass_matrix(q_num(1), q_num(2), q_num(3), q_num(4), ...
                    params.m_w, params.m_b, params.m_m, params.m_a, params.m_f, ...
                    params.I_w, params.I_b, params.I_m, params.I_a, params.I_f, ...
                    params.r_w, params.l_bx, params.l_bz, params.l_mx, params.l_mz, ...
                    params.l_ax, params.l_az, params.l_fx);

C_num = coriolis_matrix(q_num(1), q_num(2), q_num(3), q_num(4), ...
                        dq_num(1), dq_num(2), dq_num(3), dq_num(4), ...
                        params.m_w, params.m_b, params.m_m, params.m_a, params.m_f, ...
                        params.I_w, params.I_b, params.I_m, params.I_a, params.I_f, ...
                        params.r_w, params.l_bx, params.l_bz, params.l_mx, params.l_mz, ...
                        params.l_ax, params.l_az, params.l_fx);

G_num = gravity_vector(q_num(1), q_num(2), q_num(3), q_num(4), ...
                       params.m_w, params.m_b, params.m_m, params.m_a, params.m_f, ...
                       params.g, params.r_w, params.l_bx, params.l_bz, params.l_mx, params.l_mz, ...
                       params.l_ax, params.l_az, params.l_fx);

fprintf('Numerical Mass Matrix at example configuration:\n');
disp(M_num);

fprintf('Numerical Coriolis Matrix at example configuration:\n');
disp(C_num);

fprintf('Numerical Gravity Vector at example configuration:\n');
disp(G_num);

%% SAVE WORKSPACE
fprintf('\nSaving workspace to robot_dynamics.mat...\n');
save('robot_dynamics.mat', 'M', 'C', 'G', 'params', 'q', 'dq');

fprintf('\nDone! Adjust the numerical parameters in the script as needed.\n');
fprintf('Use the generated functions for simulation and control.\n');