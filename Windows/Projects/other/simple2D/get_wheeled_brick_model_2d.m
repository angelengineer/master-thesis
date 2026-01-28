function [model, model_params, helpers] = get_wheeled_brick_model_2d(varargin)
    import casadi.* %% ============================================
    %% 1. PARÁMETROS FÍSICOS
    %% ============================================
    m_b = 16.0;        % masa del brick [kg]
    m_w = 1.0;         % masa de cada rueda [kg]
    R = 0.2;           % radio de rueda [m]
    d = 0.5;           % distancia eje -> CM brick (altura) [m]
    L_w = 0.6;         % ANCHO DE VÍA (distancia entre ruedas) [m]
    g = 9.81;          % gravedad [m/s²]
    
    % Inercias
    ancho_brick = 0.8;
    prof_brick = 0.4;  % profundidad
    alto_brick = 1.0;
    
    % Inercia de Pitch (la misma de antes)
    I_b_pitch = (1/12)*m_b*(ancho_brick^2 + alto_brick^2);
    I_w = 0.5*m_w*R^2;
    
    % Inercia de Yaw (Giro sobre eje Z) - Aprox
    I_b_yaw = (1/12)*m_b*(ancho_brick^2 + prof_brick^2); 
    % Inercia total rotacional vista desde el centro del eje (simplificada)
    I_z_total = I_b_yaw + m_b*0.1^2 + 2*m_w*(L_w/2)^2; 

    %% ============================================
    %% 2. VARIABLES DEL ESTADO (7 estados)
    %% ============================================
    x_pos = SX.sym('x_pos');   % Posición global X
    y_pos = SX.sym('y_pos');   % Posición global Y
    alpha = SX.sym('alpha');   % Ángulo de Yaw (dirección)
    beta  = SX.sym('beta');    % Ángulo de Pitch (balance)
    
    v     = SX.sym('v');       % Velocidad lineal de avance
    omega = SX.sym('omega');   % Velocidad angular de Yaw
    beta_dot = SX.sym('beta_dot'); % Velocidad angular de Pitch
    
    % Vector de estado: [x, y, alpha, beta, v, omega, beta_dot]
    states = [x_pos; y_pos; alpha; beta; v; omega; beta_dot];
    
    % Controles (Torques izquierdo y derecho)
    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');
    controls = [tau_L; tau_R];

    %% ============================================
    %% 3. DINÁMICA (El truco simple)
    %% ============================================
    
    % --- A. Preparación de Torques ---
    % Torque que empuja/balancea (Suma)
    T_drive = tau_L + tau_R; 
    % Torque que gira (Diferencia) * (Radio efectivo de giro)
    % Fuerza en rueda = tau/R. Par = Fuerza * (L_w/2). Total = (tR-tL)/R * L_w/2
    T_turn  = (tau_R - tau_L) * (L_w / (2*R));
    
    % --- B. Dinámica Longitudinal (Copia exacta de tu 1D) ---
    % Parámetros compactos
    M_long = m_b + 2*m_w + 2*I_w/(R^2);
    C_long = m_b * d;
    I_long = m_b * d^2 + I_b_pitch;
    
    cos_beta = cos(beta);
    sin_beta = sin(beta);
    
    % Matriz de masa para [v_dot; beta_ddot]
    mass_matrix = [M_long,          C_long*cos_beta;
                   C_long*cos_beta, I_long];
                   
    rhs_long = [C_long*beta_dot^2*sin_beta + T_drive/R;
                m_b*g*d*sin_beta];
                
    accel_long = mass_matrix \ rhs_long;
    v_dot      = accel_long(1);
    beta_ddot  = accel_long(2);
    
    % --- C. Dinámica de Yaw (Giro) ---
    % Simplificación: I_z * alpha_ddot = Torque_de_giro
    omega_dot = T_turn / I_z_total;
    
    % --- D. Cinemática Global ---
    x_pos_dot = v * cos(alpha);
    y_pos_dot = v * sin(alpha);
    alpha_dot = omega;

    % Vector de derivadas
    states_dot = [x_pos_dot; y_pos_dot; alpha_dot; beta_dot; v_dot; omega_dot; beta_ddot];

    %% ============================================
    %% 4. CREAR OBJETO ACADOS
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 7, 1);
    model.u = controls;
    model.f_expl_expr = states_dot;
    model.f_impl_expr = states_dot - model.xdot;
    model.name = 'wheeled_brick_2d';
    
    %% Output Structs
    model_params = struct('L_w', L_w, 'R', R);
    
    % Helper para plotear CM (útil para ver si se cae)
    helpers = struct();
    helpers.get_cm_position = @(x, y, a, b) ...
        [x + d*sin(b)*cos(a); y + d*sin(b)*sin(a); d*cos(b)]; % X, Y, Z
end