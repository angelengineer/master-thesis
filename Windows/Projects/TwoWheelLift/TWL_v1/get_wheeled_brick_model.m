function model = get_wheeled_brick_model(varargin)

    import casadi.* 
    
    %% ============================================
    %% PARÁMETROS FÍSICOS
    %% ============================================
    m_b = 16.0;        % masa del brick [kg]
    m_w = 1.0;        % masa de cada rueda [kg]
    R = 0.2;          % radio de rueda [m]
    d = 0.5;         % distancia eje -> CM brick = l_z/2 [m]
    g = 9.81;         % gravedad [m/s²]
    
    % Momentos de inercia
    ancho_brick = 0.8;  % ancho en dirección Y [m]
    alto_brick = 1;   % altura del brick [m]
    I_b = (1/12)*m_b*(ancho_brick^2 + alto_brick^2);
    I_w = 0.5*m_w*R^2;
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    x = SX.sym('x');        % posición horizontal del EJE
    beta = SX.sym('beta');  % ángulo de pitch (β)
    x_dot = SX.sym('x_dot');
    beta_dot = SX.sym('beta_dot');
    
    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');
    
    %% ============================================
    %% PARÁMETROS COMPACTOS
    %% ============================================
    M = m_b + 2*m_w + 2*I_w/(R^2);
    C = m_b * d;
    I = m_b * d^2 + I_b;
    
    %% ============================================
    %% SISTEMA DINÁMICO
    %% ============================================
    cos_beta = cos(beta);
    sin_beta = sin(beta);
    
    % Matriz de masa
    mass_matrix = [M,       C*cos_beta;
                   C*cos_beta, I];
    
    % Vector del lado derecho
    T_total = tau_L + tau_R;
    rhs = [C*beta_dot^2*sin_beta + T_total/R;
           m_b*g*d*sin_beta];
    
    % Resolver para aceleraciones
    accel = mass_matrix \ rhs;
    x_ddot = accel(1);
    beta_ddot = accel(2);
    
    %% ============================================
    %% ESTADO Y DINÁMICA
    %% ============================================
    states = [x; beta; x_dot; beta_dot];
    states_dot = [x_dot; beta_dot; x_ddot; beta_ddot];
    controls = [tau_L; tau_R];
    
    %% ============================================
    %% MODELO PARA ACADOS (solo campos permitidos)
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 4, 1);
    model.u = controls;
    
    % Formulación explícita (recomendada para sistemas simples)
    model.f_expl_expr = states_dot;
    
    % También formulación implícita (opcional)
    model.f_impl_expr = states_dot - model.xdot;
    
    model.name = 'wheeled_brick_pitch';
    
end