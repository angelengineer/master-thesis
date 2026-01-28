function model = get_linear_actuator_model(varargin)
    % Modelo para MPC del subsistema de posición/velocidad con masa conocida
    % Basado en dinámica discreta del EKF proporcionado, convertido a tiempo continuo
    %
    % Estados: [posición; velocidad]
    % Control: F_aplicada
    % Parámetros: [masa; pitch] (tiempo-variante)
    %
    % model: Objeto AcadosModel
    % model_params: Parámetros físicos fijos
    % helpers: Funciones auxiliares para simulación y análisis
    
    import casadi.*
    
    %% ============================================
    %% PARÁMETROS FÍSICOS FIJOS
    %% ============================================
    g = 9.81;  % Gravedad [m/s²]
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    % Estados
    pos = SX.sym('pos');   % Posición [m]
    vel = SX.sym('vel');   % Velocidad [m/s]
    states = vertcat(pos, vel);
    
    % Control
    F_applied = SX.sym('F_applied');  % Fuerza de control [N]
    controls = F_applied;
    
    % Parámetros (tiempo-variante)
    mass = SX.sym('mass');    % Masa estimada por EKF [kg]
    pitch = SX.sym('pitch');  % Ángulo de pitch medido [rad]
    params = vertcat(mass, pitch);
    
    %% ============================================
    %% DINÁMICA EN TIEMPO CONTINUO
    %% ============================================
    % dx/dt = v
    % dv/dt = F_applied/mass - g*cos(pitch)
    pos_dot = vel;
    vel_dot = F_applied / mass - g * cos(pitch);
    states_dot = vertcat(pos_dot, vel_dot);
    
    %% ============================================
    %% MODELO ACADOS
    %% ============================================
    model = AcadosModel();
    model.x = states;          % Estados: [posición; velocidad]
    model.u = controls;        % Control: F_applied
    model.p = params;          % Parámetros: [masa; pitch]
    model.xdot = SX.sym('xdot', 2, 1);  % Derivadas de estados
    
    % Formulación explícita (continua)
    model.f_expl_expr = states_dot;
    
    % Formulación implícita (para integradores implícitos)
    model.f_impl_expr = model.xdot - states_dot;
    
    model.name = 'linear_actuator_model';
    

    
   
end