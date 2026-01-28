function model = get_wheeled_brick_model_2D(params)

import casadi.*

%% =========================
%% Parámetros (igual que antes)
%% =========================
if nargin < 1
    params = struct();
end

default_params = struct( ...
    'm_b', 16.0, ...
    'm_w', 1.0, ...
    'R', 0.2, ...
    'd', 0.5, ...
    'g', 9.81, ...
    'l_y', 0.8, ...
    'l_x', 0.4, ...
    'l_z', 1.0, ...
    'z_G', 0.225, ...
    'b', 0.150 ...
);

names = fieldnames(default_params);
for i = 1:length(names)
    if ~isfield(params, names{i})
        params.(names{i}) = default_params.(names{i});
    end
end

I_b_xx = (1/12)*params.m_b*(params.l_y^2 + params.l_z^2);
I_b_yy = (1/12)*params.m_b*(params.l_x^2 + params.l_z^2);
I_b_zz = (1/12)*params.m_b*(params.l_x^2 + params.l_y^2);

if ~isfield(params,'I_a')
    params.I_a = 0.5*params.m_w*params.R^2;
end
if ~isfield(params,'I_t')
    params.I_t = 0.25*params.m_w*params.R^2;
end

%% =========================
%% Estados y controles
%% =========================
X      = SX.sym('X');
Y      = SX.sym('Y');
theta  = SX.sym('theta');

phi_p     = SX.sym('phi_p');
phi_l     = SX.sym('phi_l');
phi_r     = SX.sym('phi_r');
phi_p_dot = SX.sym('phi_p_dot');
phi_l_dot = SX.sym('phi_l_dot');
phi_r_dot = SX.sym('phi_r_dot');

states = [
    X; Y; theta;
    phi_p; phi_l; phi_r;
    phi_p_dot; phi_l_dot; phi_r_dot
];

tau_l = SX.sym('tau_l');
tau_r = SX.sym('tau_r');
controls = [tau_l; tau_r];

%% =========================
%% Dinámica interna (igual que antes)
%% =========================
phi_dot = [phi_p_dot; phi_l_dot; phi_r_dot];

M11 = params.m_b*params.R^2 + 2*params.m_b*cos(phi_p)*params.R*params.z_G ...
    + params.m_b*params.z_G^2 + 2*params.I_a + I_b_yy;

M12 = params.m_b*params.R^2/2 + params.m_b*params.z_G*cos(phi_p)*params.R/2 + params.I_a;
M13 = M12;

M22 = (8*params.I_a*params.d^2 + ...
       (I_b_xx+I_b_zz+4*params.I_t)*params.R^2 + ...
       2*params.d^2*(params.m_b+2*params.m_w)*params.R^2) ...
       / (8*params.d^2);

M23 = -(params.R^2*(I_b_xx+I_b_zz+4*params.I_t ...
       -2*params.d^2*(params.m_b-2*params.m_w))) ...
       /(8*params.d^2);

M33 = M22;

M_bar = [M11 M12 M13;
         M12 M22 M23;
         M13 M23 M33];

C_bar = SX.zeros(3,3);
C_bar(1,1) = -params.m_b*params.R*phi_p_dot*params.z_G*sin(phi_p);

G_bar = [-params.m_b*params.g*params.z_G*sin(phi_p); 0; 0];

T_a = [0; tau_l; tau_r];
T_f = [0; -params.b*phi_l_dot; -params.b*phi_r_dot];

phi_ddot = M_bar \ (T_a + T_f - C_bar*phi_dot - G_bar);

%% =========================
%% Cinemática externa
%% =========================
v     = (params.R/2)*(phi_l_dot + phi_r_dot);
omega = (params.R/(2*params.d))*(phi_r_dot - phi_l_dot);

X_dot     = v*cos(theta);
Y_dot     = v*sin(theta);
theta_dot = omega;

%% =========================
%% Sistema completo
%% =========================
states_dot = [
    X_dot;
    Y_dot;
    theta_dot;
    phi_dot;
    phi_ddot
];

%% =========================
%% Modelo acados
%% =========================
model = AcadosModel();
model.x = states;
model.xdot = SX.sym('xdot', length(states), 1);
model.u = controls;

model.f_expl_expr = states_dot;
model.f_impl_expr = states_dot - model.xdot;
model.name = 'twinbot_pose_model';

end
