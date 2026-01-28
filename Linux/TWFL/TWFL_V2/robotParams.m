%% Robot body
Lx_b = 0.3;
Ly_b = 0.8;
Lz_b = 1;
M_b = 16;
% Robot body cdg
cdg_b = [Lx_b / 2, Ly_b / 2, Lz_b / 2]; 
%% Wheels
M_w = 1;
r_w = 0.2; %radius
t_w = 0.05; %tikness
d_w = t_w/2 + Ly_b; %distance wheels

%% Lift
Lx_m = 0.1;
Ly_m = 0.3;
Lz_m = 1.5;
M_m = 5;

%% Fork
M_arm_motor = 5;
M_fork_base = 2;
M_tooth = 1;

%% 
% --- Configuración del LESO ---
omega_o = 40; % Ajusta este valor (Ancho de banda)

beta1 = 3 * omega_o;
beta2 = 3 * omega_o^2;
beta3 = omega_o^3;

%% Parametros simulacion

T_control = 1e-3;
omega_c = 5000;
omega_o = 40;

%% Constantes
g = 9.81; %gravedad
%% Parametros PD

Kp_fork = 200; % Proportional gain
Kd_fork = 100;  % Derivative gain

Kp_lift = 200; % Proportional gain
Kd_lift = 50;  % Derivative gain


%% Parametros DOB

g_fork = 2*pi*3; %cut-off angular frequency[rad/s]
g_lift = 2*pi*5; %cut-off angular frequency[rad/s]
g_reac = 2*pi*1;