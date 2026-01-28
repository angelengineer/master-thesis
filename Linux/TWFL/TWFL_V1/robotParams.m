%Robot body
lx = 0.3;
ly = 0.8;
lz = 1;
M = 16;
%Wheels
m_w = 1;
r = 0.2; %radius
t = 0.05; %tikness
m = 1; %mass
d = t/2 + ly; %distance wheels


% --- Configuración del LESO ---
omega_o = 40; % Ajusta este valor (Ancho de banda)

beta1 = 3 * omega_o;
beta2 = 3 * omega_o^2;
beta3 = omega_o^3;

% --- Parámetro de Planta Nominal ---
m_nom = 2.0; % Masa nominal
r_nom = 0.4; % Radio nominal
J_nom = m_nom * r_nom^2;
b0 = 1 / J_nom;

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

g_fork = 2*pi*5; %cut-off angular frequency[rad/s]
g_lift = 2*pi*5; %cut-off angular frequency[rad/s]
