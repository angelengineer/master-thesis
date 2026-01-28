function model = model_simplified(varargin)
  
    import casadi.*
    %% PARÁMETROS SIMBÓLICOS
    m_load = SX.sym('m_load');   % masa de la carga
    I_b    = SX.sym('I_b');      % inercia del cuerpo
    l_bx = SX.sym('l_bx');% desplazamiento horizontal del CoG [m]
    l_bz = SX.sym('l_bz');  % desplazamiento vertical del CoG [m]
    

    %% PARÁMETROS FÍSICOS
    % Masas
    m_b = 21.0 + m_load;        % masa del cuerpo principal [kg]
    m_w = 1.0;         % masa de cada rueda [kg]
    
    % Geometría
    r_w = 0.2;         % radio de rueda [m]
    
    % Gravedad
    g = 9.81;          % [m/s²]
    
    % Momentos de inercia

    % Parametro de inercia 
    I_w = 0.5*m_w*r_w^2; % [kg·m²] wheels
    
    %% VARIABLES SIMBÓLICAS
    % Estados generalizados
    x = SX.sym('x');              % posición horizontal del eje de ruedas
    theta_p = SX.sym('theta_p');  % ángulo de pitch (θ_p)
    x_dot = SX.sym('x_dot');
    theta_p_dot = SX.sym('theta_p_dot');
    
    % Controles
    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');
    
    %% CINEMÁTICA - POSICIÓN DEL CoG DEL CUERPO
    % Posición del CoG del cuerpo principal en marco mundial
    x_b = x + l_bx*cos(theta_p) + l_bz*sin(theta_p);
    z_b = r_w - l_bx*sin(theta_p) + l_bz*cos(theta_p);
    
    % Velocidades del CoG (derivadas respecto al tiempo)
    x_b_dot = x_dot - l_bx*theta_p_dot*sin(theta_p) + l_bz*theta_p_dot*cos(theta_p);
    z_b_dot = -l_bx*theta_p_dot*cos(theta_p) - l_bz*theta_p_dot*sin(theta_p);
    
    %% ============================================
    %% ENERGÍAS - SUBSISTEMA DE RUEDAS
    %% ============================================
    % Energía cinética traslacional de ambas ruedas
    T_w_trans = m_w * x_dot^2;  % factor 2 se agregará después
    
    % Energía cinética rotacional de ambas ruedas
    % theta_w_dot = x_dot / r_w (sin deslizamiento)
    T_w_rot = I_w * (x_dot/r_w)^2;  % factor 2 se agregará después
    
    % Energía potencial de las ruedas (referencia en el eje)
    U_w = 0;
    
    % Total del subsistema de ruedas (ambas ruedas)
    T_w = T_w_trans + T_w_rot;
    
    %% ============================================
    %% ENERGÍAS - SUBSISTEMA DEL CUERPO PRINCIPAL
    %% ============================================
    % Energía cinética traslacional
    T_b_trans = (1/2) * m_b * (x_b_dot^2 + z_b_dot^2);
    
    % Energía cinética rotacional
    T_b_rot = (1/2) * I_b * theta_p_dot^2;
    
    % Energía cinética total del cuerpo
    T_b = T_b_trans + T_b_rot;
    
    % Energía potencial gravitacional
    U_b = m_b * g * (-l_bx*sin(theta_p) + l_bz*cos(theta_p));
    
    %% ============================================
    %% LAGRANGIANO
    %% ============================================
    % Energías totales
    T_total = 2*T_w + T_b;
    U_total = 2*U_w + U_b;
    
    % Lagrangiano L = T - U
    L_lagrangian = T_total - U_total;
    
    %% ============================================
    %% ECUACIONES DE EULER-LAGRANGE
    %% ============================================
    % Variables generalizadas
    q = [x; theta_p];
    q_dot = [x_dot; theta_p_dot];
    
    % Derivadas parciales del Lagrangiano
    dL_dq = jacobian(L_lagrangian, q)';      % ∂L/∂q
    dL_dqdot = jacobian(L_lagrangian, q_dot)'; % ∂L/∂q̇
    
    % Derivada temporal de ∂L/∂q̇ (usando regla de la cadena)
    d_dLdqdot_dt = jacobian(dL_dqdot, q) * q_dot + ...
                   jacobian(dL_dqdot, q_dot) * SX.sym('q_ddot', 2, 1);
    
    % Extraer matriz de inercia M y vector de fuerzas C+G
    % d/dt(∂L/∂q̇) - ∂L/∂q = M*q̈ + h = τ
    % donde h contiene términos de Coriolis, centrífugos y gravitacionales
    
    % Matriz de inercia (coeficientes de q̈)
    M_matrix = jacobian(dL_dqdot, q_dot);
    
    % Vector h = d/dt(∂L/∂q̇)|_{q̈=0} - ∂L/∂q
    dL_dqdot_expanded = jacobian(dL_dqdot, q) * q_dot;
    h_vector = dL_dqdot_expanded - dL_dq;
    
    %% ============================================
    %% TORQUES GENERALIZADOS
    %% ============================================
    % τ_total aplicado a través de las ruedas
    tau_total = tau_L + tau_R;
    tau_generalized = [tau_total/r_w; 0];  % [Q_x; Q_θ]
    
    %% ============================================
    %% RESOLVER PARA ACELERACIONES
    %% ============================================
    % M*q̈ = τ - h
    q_ddot = M_matrix \ (tau_generalized - h_vector);
    x_ddot = q_ddot(1);
    theta_p_ddot = q_ddot(2);
    
    %% ============================================
    %% ESTADO Y DINÁMICA PARA ACADOS
    %% ============================================
    states = [x; theta_p; x_dot; theta_p_dot];
    states_dot = [x_dot; theta_p_dot; x_ddot; theta_p_ddot];
    controls = [tau_L; tau_R];
    
    %% ============================================
    %% MODELO ACADOS
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 4, 1);
    model.u = controls;
    
    % Formulación explícita
    model.f_expl_expr = states_dot;
    
    % Formulación implícita
    model.f_impl_expr = states_dot - model.xdot;
    
        % Parámetros del modelo
    model.p = [m_load; I_b; l_bx; l_bz];
    
    model.name = mfilename;
   
end