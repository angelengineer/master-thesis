function [model, helpers] = get_model(varargin)

import casadi.*

%% ========================
%% PARÁMETROS FÍSICOS
%% ========================
bx = 0.3; by = 0.8; bz = 1.0;
m_b = 16.0;

m_L = 10.0; lx = 0.1; ly = 0.3; lz = 1.5;

m_f_base = 3.0;
m_f_tooth = 1.0;
fxb = 0.1;
fxt = 0.6;

m_w = 1.0;
R = 0.2;
g = 9.81;

m_total = m_b + m_L + m_f_base + 2*m_f_tooth + 2*m_w;

%% ========================
%% VARIABLES SIMBÓLICAS
%% ========================
x = SX.sym('x');
beta = SX.sym('beta');
x_dot = SX.sym('x_dot');
beta_dot = SX.sym('beta_dot');

tau_L = SX.sym('tau_L');
tau_R = SX.sym('tau_R');

% parámetro acados (vector)
p = SX.sym('p',1);
d_f = p(1);

%% ========================
%% CENTRO DE MASA
%% ========================
cm_brick_x = 0;
cm_brick_z = R + bz/2;

cm_actuador_x = -bx/2 - lx/2;
cm_actuador_z = R + bz/2;

cm_fork_base_x = -bx/2 + fxb/2;
cm_fork_base_z = R + bz + d_f;

cm_fork_teeth_x = -bx/2 + fxb + fxt/2;
cm_fork_teeth_z = R + bz + d_f;

cm_wheels_x = 0;
cm_wheels_z = R;

cm_x = (m_b*cm_brick_x + m_L*cm_actuador_x + ...
        m_f_base*cm_fork_base_x + 2*m_f_tooth*cm_fork_teeth_x) / m_total;

cm_z = (m_b*cm_brick_z + m_L*cm_actuador_z + ...
        m_f_base*cm_fork_base_z + 2*m_f_tooth*cm_fork_teeth_z + ...
        2*m_w*cm_wheels_z) / m_total;

d = cm_z - R;

%% ========================
%% INERCIAS
%% ========================
I_b_cm = (1/12)*m_b*(by^2 + bz^2);
I_L_cm = (1/12)*m_L*(ly^2 + lz^2);
I_fb_cm = (1/12)*m_f_base*(fxb^2 + by^2);
I_ft_cm = (1/12)*m_f_tooth*(fxt^2);
I_w = 0.5*m_w*R^2;

dist_brick_sq = (cm_brick_x - cm_x)^2 + (cm_brick_z - cm_z)^2;
dist_actuador_sq = (cm_actuador_x - cm_x)^2 + (cm_actuador_z - cm_z)^2;
dist_fork_base_sq = (cm_fork_base_x - cm_x)^2 + (cm_fork_base_z - cm_z)^2;
dist_fork_teeth_sq = (cm_fork_teeth_x - cm_x)^2 + (cm_fork_teeth_z - cm_z)^2;

I_total = I_b_cm + m_b*dist_brick_sq + ...
          I_L_cm + m_L*dist_actuador_sq + ...
          I_fb_cm + m_f_base*dist_fork_base_sq + ...
          2*(I_ft_cm + m_f_tooth*dist_fork_teeth_sq);

%% ========================
%% DINÁMICA (SIN \ )
%% ========================
M = m_total + 2*I_w/R^2;
C = m_total*d;

cosb = cos(beta);
sinb = sin(beta);

M11 = M;
M12 = C*cosb;
M22 = I_total;

rhs1 = C*beta_dot^2*sinb + (tau_L + tau_R)/R;
rhs2 = m_total*g*d*sinb;

detM = M11*M22 - M12^2;

x_ddot = ( rhs1*M22 - rhs2*M12 ) / detM;
beta_ddot = ( M11*rhs2 - M12*rhs1 ) / detM;

%% ========================
%% ESTADOS
%% ========================
states = [x; beta; x_dot; beta_dot];
states_dot = [x_dot; beta_dot; x_ddot; beta_ddot];
controls = [tau_L; tau_R];

%% ========================
%% MODELO ACADOS
%% ========================
model = AcadosModel();
model.x = states;
model.xdot = SX.sym('xdot',4);
model.u = controls;
model.p = p;

model.f_expl_expr = states_dot;
model.f_impl_expr = states_dot - model.xdot;
model.name = 'wheeled_brick_fork_param';

%% helpers (igual que antes)
helpers.f_expl = Function('f_expl',{states,controls,p},{states_dot});

end
