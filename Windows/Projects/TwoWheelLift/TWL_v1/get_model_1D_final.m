function [model, helpers] = get_model_1D_final(varargin)

import casadi.*

%% ========================
%% PARÁMETROS FÍSICOS
%% ========================
% Body (brick)
bx = 0.3; by = 0.8; bz = 1.0;
m_b = 16.0;

% Actuador lineal
m_L = 5.0; lx = 0.1; ly = 0.3; lz = 1.5;

% Fork (herramienta)
m_f_base = 3.0;    % masa base del fork
m_f_tooth = 1.0;   % masa de un diente
fxb = 0.1;         % longitud en x de la base
fxt = 0.6;         % longitud en x de un diente
fzt = 0.02;        % altura en z de los dientes
fyt = 0.1;         % ancho en y de los dientes

% Ruedas
m_w = 1.0;
R = 0.2;
g = 9.81;

% Masas totales
m_body = m_b + m_L + m_f_base + 2*m_f_tooth;  % masa del cuerpo inclinable
m_total = m_body + 2*m_w;                     % masa total del sistema

%% ========================
%% VARIABLES SIMBÓLICAS
%% ========================
x = SX.sym('x');          % posición horizontal del robot
beta = SX.sym('beta');    % ángulo de inclinación del cuerpo (0 = vertical)
x_dot = SX.sym('x_dot');
beta_dot = SX.sym('beta_dot');

% Control: par aplicado a AMBAS ruedas (movimiento 1D simétrico)
tau = SX.sym('tau');

% Parámetro: extensión del actuador
p = SX.sym('p', 1);
d_f = p(1);

%% ========================
%% GEOMETRÍA - COORDENADAS LOCALES
%% ========================
% Sistema local del cuerpo:
% - Origen: eje de las ruedas (punto de contacto + R en z)
% - x_local: hacia adelante del cuerpo (cuando beta=0)
% - z_local: hacia arriba del cuerpo (cuando beta=0)

% 1. Body (brick) - CM en el centro geométrico
cm_b_local_x = 0;
cm_b_local_z = bz/2;

% 2. Actuador lineal - montado en la parte trasera
%    Base del actuador en la parte trasera del body
cm_L_local_x = -bx/2 - lx/2;  % centro del actuador en x
cm_L_local_z = bz;          % centro del actuador en z (longitud fija lz)

% 3. Base del fork - se acopla al extremo del actuador
%    La base se extiende desde el extremo del actuador
cm_fb_local_x = -bx/2 + fxb/2;  % centro de la base en x
cm_fb_local_z = lz + d_f;     % centro de la base en z (se extiende d_f)

% 4. Dientes del fork (2 dientes simétricos)
%    Se extienden desde el extremo de la base
cm_ft_local_x = -bx/2 + fxb + fxt/2;  % centro de un diente en x
cm_ft_local_z = lz + d_f;           % centro de un diente en z (misma altura que la base)

% Nota: cuando d_f=0, el fork está completamente contraído
%       y su posición z coincide con el extremo del actuador (lz)

%% ========================
%% TRANSFORMACIÓN A COORDENADAS GLOBALES
%% ========================
% Punto de articulación (eje de las ruedas)
x_axle = x;
z_axle = R;

% Matriz de rotación [cosβ -sinβ; sinβ cosβ]
cosb = cos(beta);
sinb = sin(beta);

% Transformación de coordenadas locales a globales:
% [x_global] = [x_axle] + [cosβ -sinβ][x_local]
% [z_global]   [z_axle]   [sinβ  cosβ][z_local]

% Body (brick)
cm_b_x = x_axle + cm_b_local_x*cosb - cm_b_local_z*sinb;
cm_b_z = z_axle + cm_b_local_x*sinb + cm_b_local_z*cosb;

% Actuador
cm_L_x = x_axle + cm_L_local_x*cosb - cm_L_local_z*sinb;
cm_L_z = z_axle + cm_L_local_x*sinb + cm_L_local_z*cosb;

% Base del fork
cm_fb_x = x_axle + cm_fb_local_x*cosb - cm_fb_local_z*sinb;
cm_fb_z = z_axle + cm_fb_local_x*sinb + cm_fb_local_z*cosb;

% Dientes del fork (2 dientes simétricos)
% Diente derecho
cm_ft1_x = x_axle + cm_ft_local_x*cosb - cm_ft_local_z*sinb;
cm_ft1_z = z_axle + cm_ft_local_x*sinb + cm_ft_local_z*cosb;
% Diente izquierdo (simétrico en x)
cm_ft2_x = x_axle + (-cm_ft_local_x)*cosb - cm_ft_local_z*sinb;
cm_ft2_z = z_axle + (-cm_ft_local_x)*sinb + cm_ft_local_z*cosb;

% Ruedas (no rotan con el cuerpo)
cm_w_x = x_axle;
cm_w_z = R;

%% ========================
%% CENTRO DE MASA TOTAL
%% ========================
cm_x_total = (m_b*cm_b_x + m_L*cm_L_x + m_f_base*cm_fb_x + ...
              m_f_tooth*cm_ft1_x + m_f_tooth*cm_ft2_x + 2*m_w*cm_w_x) / m_total;
cm_z_total = (m_b*cm_b_z + m_L*cm_L_z + m_f_base*cm_fb_z + ...
              m_f_tooth*cm_ft1_z + m_f_tooth*cm_ft2_z + 2*m_w*cm_w_z) / m_total;

% Distancia del CM total al eje de rotación (para torque gravitacional)
dx_cm = cm_x_total - x_axle;
dz_cm = cm_z_total - z_axle;

%% ========================
%% INERCIAS
%% ========================
% Inercias propias de cada componente respecto a su CM
I_b_cm = (1/12)*m_b*(by^2 + bz^2);
I_L_cm = (1/12)*m_L*(ly^2 + lz^2);
I_fb_cm = (1/12)*m_f_base*(fxb^2 + by^2);
I_ft_cm = (1/12)*m_f_tooth*(fxt^2 + fyt^2);

% Teorema de ejes paralelos: I_total = I_cm + m*d^2
% donde d es la distancia del CM del componente al CM total
dist_b_sq = (cm_b_x - cm_x_total)^2 + (cm_b_z - cm_z_total)^2;
dist_L_sq = (cm_L_x - cm_x_total)^2 + (cm_L_z - cm_z_total)^2;
dist_fb_sq = (cm_fb_x - cm_x_total)^2 + (cm_fb_z - cm_z_total)^2;
dist_ft1_sq = (cm_ft1_x - cm_x_total)^2 + (cm_ft1_z - cm_z_total)^2;
dist_ft2_sq = (cm_ft2_x - cm_x_total)^2 + (cm_ft2_z - cm_z_total)^2;

% Inercia total del cuerpo inclinable respecto al CM total
I_body_cm = I_b_cm + m_b*dist_b_sq + ...
            I_L_cm + m_L*dist_L_sq + ...
            I_fb_cm + m_f_base*dist_fb_sq + ...
            I_ft_cm + m_f_tooth*dist_ft1_sq + ...
            I_ft_cm + m_f_tooth*dist_ft2_sq;

% Inercia de las ruedas (traslación + rotación)
I_w = 0.5*m_w*R^2;  % inercia de una rueda respecto a su eje
I_w_equiv = 2*(I_w/R^2 + m_w);  % inercia equivalente de ambas ruedas

%% ========================
%% ECUACIONES DE MOVIMIENTO (NEWTON-EULER)
%% ========================
% Matriz de masa para el sistema [x, β]
M11 = m_total + I_w_equiv;  % masa total + inercia equivalente de ruedas
M12 = m_body * dz_cm;       % acoplamiento inercial (aproximado)
M21 = M12;
M22 = I_body_cm + m_body*(dz_cm^2 + dx_cm^2);  % inercia del cuerpo respecto al eje

M = [M11, M12; M21, M22];

% Vector de fuerzas (lado derecho de las ecuaciones)
% Fuerza centrífuga + fuerza de control + gravedad
F1 = m_body * dz_cm * beta_dot^2 + (2*tau)/R;  % 2τ/R para ambas ruedas
F2 = m_body * g * dx_cm;  % torque gravitacional (m*g*d*sinβ aproximado)

% Resolver para aceleraciones: M * [x_ddot; β_ddot] = [F1; F2]
accel = M \ [F1; F2];
x_ddot = accel(1);
beta_ddot = accel(2);

%% ========================
%% ESTADOS Y MODELO ACADOS
%% ========================
states = [x; beta; x_dot; beta_dot];
states_dot = [x_dot; beta_dot; x_ddot; beta_ddot];
controls = tau;

model = AcadosModel();
model.x = states;
model.xdot = SX.sym('xdot', 4);
model.u = controls;
model.p = p;

model.f_expl_expr = states_dot;
model.f_impl_expr = states_dot - model.xdot;
model.name = 'wheeled_robot_fork_1D';

%% ========================
%% HELPERS PARA ANÁLISIS
%% ========================
helpers.f_expl = Function('f_expl', {states, controls, p}, {states_dot});
helpers.cm_total = Function('cm_total', {states, p}, {[cm_x_total; cm_z_total]});
helpers.M_matrix = Function('M_matrix', {states, p}, {M});
helpers.geometry = Function('geometry', {states, p}, {...
    [cm_b_x; cm_b_z], ...      % body
    [cm_L_x; cm_L_z], ...      % actuator
    [cm_fb_x; cm_fb_z], ...    % fork base
    [cm_ft1_x; cm_ft1_z], ...  % tooth 1
    [cm_ft2_x; cm_ft2_z] ...   % tooth 2
});

end