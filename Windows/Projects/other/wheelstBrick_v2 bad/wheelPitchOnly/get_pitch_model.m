function model = get_pitch_model(varargin)

    % Modelo del wheeled brick SIN posición absoluta x
    % Frame local: x = 0 por definición
    % Estados: [beta, x_dot, beta_dot]

    import casadi.*

    %% ============================================
    %% PARÁMETROS FÍSICOS
    %% ============================================
    m_b = 16.0;        % masa del brick [kg]
    m_w = 1.0;         % masa de cada rueda [kg]
    R = 0.2;           % radio de rueda [m]
    d = 0.5;           % distancia eje -> CM brick [m]
    g = 9.81;          % gravedad [m/s^2]

    % Momentos de inercia
    ancho_brick = 0.8; % ancho en Y [m]
    alto_brick  = 1.0; % altura [m]
    I_b = (1/12)*m_b*(ancho_brick^2 + alto_brick^2);
    I_w = 0.5*m_w*R^2;

    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    beta      = SX.sym('beta');       % ángulo de pitch
    x_dot     = SX.sym('x_dot');      % velocidad longitudinal
    beta_dot  = SX.sym('beta_dot');   % velocidad angular

    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');

    %% ============================================
    %% PARÁMETROS COMPACTOS
    %% ============================================
    M = m_b + 2*m_w + 2*I_w/(R^2);
    C = m_b * d;
    I = m_b * d^2 + I_b;

    %% ============================================
    %% DINÁMICA
    %% ============================================
    cos_beta = cos(beta);
    sin_beta = sin(beta);

    % Matriz de masa
    mass_matrix = [ M,            C*cos_beta;
                    C*cos_beta,   I           ];

    % Entrada total
    T_total = tau_L + tau_R;

    % Lado derecho
    rhs = [ C*beta_dot^2*sin_beta + T_total/R;
            m_b*g*d*sin_beta                     ];

    % Aceleraciones
    accel = mass_matrix \ rhs;
    x_ddot    = accel(1);
    beta_ddot = accel(2);

    %% ============================================
    %% ESTADOS Y ECUACIONES
    %% ============================================
    states = [beta; x_dot; beta_dot];
    states_dot = [beta_dot;
                  x_ddot;
                  beta_ddot];

    controls = [tau_L; tau_R];

    %% ============================================
    %% MODELO ACADOS
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 3, 1);
    model.u = controls;

    model.f_expl_expr = states_dot;
    model.f_impl_expr = states_dot - model.xdot;

    model.name = 'wheeled_brick_pitch_no_x';

end