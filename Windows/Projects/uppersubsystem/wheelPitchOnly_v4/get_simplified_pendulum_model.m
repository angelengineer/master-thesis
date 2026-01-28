function [model, model_params, helpers] = get_simplified_pendulum_model(varargin)
    % Modelo del péndulo invertido usando formulación Lagrangiana
    %   Basado en energías cinética y potencial
    %   
    %   model: Objeto AcadosModel
    %   model_params: Estructura con parámetros físicos
    %   helpers: Funciones auxiliares
    
    import casadi.*
    
    %% ============================================
    %% PARÁMETROS FÍSICOS
    %% ============================================
    % Masas
    m_b = 21.0;        % masa del cuerpo principal [kg]
    m_w = 1.0;         % masa de cada rueda [kg]
    
    % Geometría
    r_w = 0.2;         % radio de rueda [m]
    l_bx = -0.0232;       % desplazamiento horizontal del CoG [m]
    l_bz = 0.8750;        % desplazamiento vertical del CoG [m]
    
    % Gravedad
    g = 9.81;          % [m/s²]
    
    % Momentos de inercia
    ancho_brick = 0.8;  % ancho en dirección Y [m]
    alto_brick = 1.0;   % altura del brick [m]
    I_b = (1/12)*m_b*(ancho_brick^2 + alto_brick^2);
    I_b = 8.41;        % [kg·m²]
    I_w = 0.5*m_w*r_w^2; % [kg·m²]
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    % Estados generalizados
    x = SX.sym('x');              % posición horizontal del eje de ruedas
    theta_p = SX.sym('theta_p');  % ángulo de pitch (θ_p)
    x_dot = SX.sym('x_dot');
    theta_p_dot = SX.sym('theta_p_dot');
    
    % Controles
    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');
    
    %% ============================================
    %% CINEMÁTICA - POSICIÓN DEL CoG DEL CUERPO
    %% ============================================
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
    
    model.name = 'simplified_inverted_pendulum';
    
    %% ============================================
    %% PARÁMETROS
    %% ============================================
    model_params = struct();
    model_params.m_b = m_b;
    model_params.m_w = m_w;
    model_params.r_w = r_w;
    model_params.l_bx = l_bx;
    model_params.l_bz = l_bz;
    model_params.g = g;
    model_params.I_b = I_b;
    model_params.I_w = I_w;
    
    % Parámetros derivados
    model_params.M_matrix = Function('M_matrix', {q, q_dot}, {M_matrix});
    model_params.h_vector = Function('h_vector', {q, q_dot}, {h_vector});
    
    %% ============================================
    %% FUNCIONES AUXILIARES
    %% ============================================
    helpers = struct();
    
    % Posición del CoG del cuerpo
    helpers.get_body_cog = Function('get_body_cog', ...
        {x, theta_p}, {x_b, z_b}, ...
        {'x', 'theta_p'}, {'x_b', 'z_b'});
    
    % Velocidad del CoG del cuerpo
    helpers.get_body_velocity = Function('get_body_velocity', ...
        {x, theta_p, x_dot, theta_p_dot}, {x_b_dot, z_b_dot}, ...
        {'x', 'theta_p', 'x_dot', 'theta_p_dot'}, {'x_b_dot', 'z_b_dot'});
    
    % Energías del sistema
    helpers.get_energies = Function('get_energies', ...
        {states}, {T_total, U_total, L_lagrangian}, ...
        {'states'}, {'T', 'U', 'L'});
    
    % Dinámica completa
    helpers.f_expl = Function('f_expl', ...
        {states, controls}, {states_dot}, ...
        {'states', 'controls'}, {'states_dot'});
    
    % Matriz de inercia y vector h
    helpers.M_func = Function('M', {states}, {M_matrix});
    helpers.h_func = Function('h', {states}, {h_vector});
    
    %% ============================================
    %% INFORMACIÓN
    %% ============================================
    if nargin == 0 || ~strcmp(varargin{1}, 'silent')
        fprintf('=============================================\n');
        fprintf('MODELO LAGRANGIANO - PÉNDULO INVERTIDO\n');
        fprintf('=============================================\n');
        fprintf('\nVariables de estado:\n');
        fprintf('  x: posición del eje de ruedas [m]\n');
        fprintf('  θ_p: ángulo de pitch [rad]\n');
        fprintf('  ẋ, θ̇_p: velocidades [m/s, rad/s]\n');
        fprintf('\nControles:\n');
        fprintf('  τ_L, τ_R: torques en ruedas izq/der [Nm]\n');
        fprintf('\nParámetros físicos:\n');
        fprintf('  m_b = %.2f kg, m_w = %.2f kg\n', m_b, m_w);
        fprintf('  r_w = %.3f m\n', r_w);
        fprintf('  l_bx = %.3f m, l_bz = %.3f m\n', l_bx, l_bz);
        fprintf('  I_b = %.4f kg·m², I_w = %.4f kg·m²\n', I_b, I_w);
        fprintf('\nFormulación:\n');
        fprintf('  - Basada en Lagrangiano L = T - U\n');
        fprintf('  - Ecuaciones de Euler-Lagrange\n');
        fprintf('  - Forma: M(q)q̈ + h(q,q̇) = τ\n');
        fprintf('=============================================\n');
    end
end