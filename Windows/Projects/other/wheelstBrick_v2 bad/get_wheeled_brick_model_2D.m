function model = get_wheeled_brick_model_2D(varargin)
    % Sistema BrickBot 2D COMPLETO - implementación consistente
    % Estados: [x, y, α, v, v_lat, β, ω, β_dot]
    % Controles: [τ_L, τ_R]

    import casadi.*

    %% ============================================
    %% PARÁMETROS FÍSICOS
    %% ============================================
    m_b = 16.0;
    m_w = 1.0;
    R = 0.2;
    d = 0.5;
    g = 9.81;
    l_y = 0.8;

    l_x = 0.4;
    l_z = 1.0;

    I_b_xx = (1/12)*m_b*(l_y^2 + l_z^2);
    I_b_yy = (1/12)*m_b*(l_x^2 + l_z^2);
    I_b_zz = (1/12)*m_b*(l_x^2 + l_y^2);

    I_w = 0.5*m_w*R^2;

    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    x = SX.sym('x');
    y = SX.sym('y');
    alpha = SX.sym('alpha');
    v = SX.sym('v');
    v_lat = SX.sym('v_lat');
    beta = SX.sym('beta');
    omega = SX.sym('omega');
    beta_dot = SX.sym('beta_dot');

    states = [x; y; alpha; v; v_lat; beta; omega; beta_dot];
    states_dot = SX.sym('states_dot', 8, 1);

    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');
    controls = [tau_L; tau_R];

    %% ============================================
    %% PARÁMETROS COMPACTOS
    %% ============================================
    M_long = m_b + 2*m_w + 2*I_w/R^2;
    M_lat  = m_b + 2*m_w;
    C = m_b*d;

    %% ============================================
    %% MATRIZ DE MASA REDUCIDA
    %% ============================================
    M_red = SX.zeros(4,4);

    I_beta_eff = I_b_yy + m_b*d^2 + 2*I_w;
    I_omega_eff = I_b_zz*cos(beta)^2 + I_b_xx*sin(beta)^2 + ...
                  m_b*d^2*sin(beta)^2 + 2*m_w*(l_y/2)^2 + ...
                  2*I_w*(l_y/(2*R))^2;

    % v̇
    M_red(1,1) = M_long;
    M_red(1,3) = C*cos(beta);

    % v̇_lat
    M_red(2,2) = M_lat;
    M_red(2,3) = C*sin(beta);

    % β̈
    M_red(3,1) = C*cos(beta);
    M_red(3,2) = C*sin(beta);
    M_red(3,3) = I_beta_eff;

    % ω̇  (acoplamientos CORRECTOS)
    M_red(4,1) =  C*sin(beta);      % + m_b d sinβ v̇
    M_red(4,2) = -C*cos(beta);      % - m_b d cosβ v̇_lat
    M_red(4,4) = I_omega_eff;

    %% ============================================
    %% TÉRMINOS NO LINEALES
    %% ============================================
    F_red = SX.zeros(4,1);

    F_red(1) =  M_lat*v_lat*omega + C*sin(beta)*beta_dot^2;
    F_red(2) = -M_lat*v*omega - C*cos(beta)*beta_dot*omega;

    F_red(3) = (I_b_xx - I_b_zz)*omega^2*sin(beta)*cos(beta) ...
               - m_b*g*d*sin(beta) ...
               - C*sin(beta)*v_lat*omega;

    F_red(4) = -2*(I_b_xx - I_b_zz + m_b*d^2)*beta_dot*omega*sin(beta)*cos(beta) ...
               - 2*C*cos(beta)*beta_dot*v;

    %% ============================================
    %% MATRIZ DE ENTRADA
    %% ============================================
    B_red = SX.zeros(4,2);
    B_red(1,:) = [1/R, 1/R];
    B_red(4,:) = [-l_y/(2*R), l_y/(2*R)];

    %% ============================================
    %% RESOLUCIÓN DINÁMICA (CORREGIDA)
    %% ============================================
    acc = M_red \ (F_red + B_red*controls);

    v_dot       = acc(1);
    v_lat_dot   = acc(2);
    beta_ddot   = acc(3);
    omega_dot   = acc(4);

    %% ============================================
    %% CINEMÁTICA
    %% ============================================
    x_dot     = v*cos(alpha) - v_lat*sin(alpha);
    y_dot     = v*sin(alpha) + v_lat*cos(alpha);
    alpha_dot = omega;

    %% ============================================
    %% SISTEMA COMPLETO
    %% ============================================
    states_dot_expr = [x_dot;
                       y_dot;
                       alpha_dot;
                       v_dot;
                       v_lat_dot;
                       beta_dot;
                       omega_dot;
                       beta_ddot];

    %% ============================================
    %% MODELO ACADOS
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = states_dot;
    model.u = controls;

    model.f_expl_expr = states_dot_expr;
    model.f_impl_expr = states_dot_expr - states_dot;

    model.name = 'wheeled_brick_2D_complete';
end
