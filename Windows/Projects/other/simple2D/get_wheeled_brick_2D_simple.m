function [model, model_params, helpers] = get_wheeled_brick_2D_simple(varargin)
    import casadi.*
    
    %% PARÁMETROS FÍSICOS (extendidos)
    m_b = 16.0;        % masa brick [kg]
    m_w = 1.0;         % masa por rueda [kg]
    R = 0.2;           % radio rueda [m]
    d = 0.5;           % distancia eje-CM [m]
    g = 9.81;          % gravedad
    
    % Nuevos parámetros para 2D
    w = 0.4;           % mitad del ancho de vía [m] (distancia lateral rueda-eje)
    l_x = 0.5;         % largo brick en dirección x [m]
    l_y = 0.8;         % ancho brick [m] (igual que ancho_brick original)
    
    % Momentos de inercia
    I_b_pitch = (1/12)*m_b*(l_y^2 + 1^2);  % Inercia pitch (original)
    I_b_yaw = (1/12)*m_b*(l_x^2 + l_y^2);  % Inercia yaw (nuevo)
    I_w = 0.5*m_w*R^2;                     % Inercia rueda
    I_zz = I_b_yaw + 2*m_w*w^2;            % Inercia total eje Z (yaw)
    
    %% VARIABLES SIMBÓLICAS (7 estados, 2 controles)
    x = SX.sym('x');          % posición global X
    y = SX.sym('y');          % posición global Y
    alpha = SX.sym('alpha');  % yaw (ángulo de orientación)
    beta = SX.sym('beta');    % pitch (inclinación)
    v = SX.sym('v');          % velocidad lineal cuerpo (dirección heading)
    omega = SX.sym('omega');  % velocidad angular yaw
    beta_dot = SX.sym('beta_dot');
    
    tau_L = SX.sym('tau_L');  % torque rueda izquierda
    tau_R = SX.sym('tau_R');  % torque rueda derecha
    
    %% DINÁMICA GLOBAL (cinemática)
    x_dot = v * cos(alpha);
    y_dot = v * sin(alpha);
    alpha_dot = omega;
    
    %% DINÁMICA LOCAL (aceleraciones cuerpo)
    % Parámetros compactos (iguales al modelo 1D)
    M = m_b + 2*m_w + 2*I_w/(R^2);
    C = m_b * d;
    I_pitch = m_b * d^2 + I_b_pitch;
    
    % Sistema acoplado pitch/velocidad lineal (igual que 1D)
    T_total = tau_L + tau_R;
    mass_matrix = [M,          C*cos(beta);
                   C*cos(beta), I_pitch];
    rhs = [C*beta_dot^2*sin(beta) + T_total/R;
           m_b*g*d*sin(beta)];
    accel = mass_matrix \ rhs;
    v_dot = accel(1);
    beta_ddot = accel(2);
    
    %% DINÁMICA YAW (nueva)
    tau_yaw = (tau_R - tau_L) * w;  % Torque neto en yaw
    omega_dot = tau_yaw / I_zz;
    
    %% ENSAMBLAR MODELO
    states = [x; y; alpha; beta; v; omega; beta_dot];
    states_dot = [x_dot; y_dot; alpha_dot; beta_dot; v_dot; omega_dot; beta_ddot];
    controls = [tau_L; tau_R];
    
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 7, 1);
    model.u = controls;
    model.f_expl_expr = states_dot;
    model.f_impl_expr = states_dot - model.xdot;
    model.name = 'wheeled_brick_2D';
    
    %% PARÁMETROS Y HELPERS (extendidos)
    model_params = struct();
    model_params.m_b = m_b; model_params.w = w; % ... (todos los parámetros)
    
    helpers = struct();
    helpers.get_cm_position = @(x_val, y_val, alpha_val, beta_val) ...
        [x_val + d*sin(beta_val)*cos(alpha_val); ...
         y_val + d*sin(beta_val)*sin(alpha_val); ...
         R + d*cos(beta_val)]; % Posición CM en 3D
end