function [model, model_params, helpers] = get_wheeled_brick_model(varargin)
    % Sistema BrickBot 2D  con nueva notación
    % Salidas:
    %   model: Objeto AcadosModel (solo campos permitidos)
    %   model_params: Estructura separada con parámetros
    %   helpers: Estructura con funciones auxiliares
    
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
    
    model.name = 'wheeled_brick_2D';
    
    %% ============================================
    %% PARÁMETROS EN ESTRUCTURA SEPARADA
    %% ============================================
    model_params = struct();
    model_params.m_b = m_b;
    model_params.m_w = m_w;
    model_params.R = R;
    model_params.d = d;
    model_params.g = g;
    model_params.I_b = I_b;
    model_params.I_w = I_w;
    model_params.M = M;
    model_params.C = C;
    model_params.I_eff = I;
    
    %% ============================================
    %% FUNCIONES AUXILIARES EN ESTRUCTURA SEPARADA
    %% ============================================
    helpers = struct();
    
    % Función para calcular ángulos de ruedas
    helpers.get_wheel_angles = @(x_val, x0, theta_L0, theta_R0) ...
        [theta_L0 + (x_val - x0)/R;
         theta_R0 + (x_val - x0)/R];
    
    % Función para posición del CM relativa al eje
    helpers.get_cm_position = @(x_val, beta_val) ...
        [x_val + d*sin(beta_val);
         d*cos(beta_val)];
    
    % Función para posición absoluta sobre el suelo
    helpers.get_absolute_position = @(x_val, beta_val) ...
        [x_val + d*sin(beta_val);
         R + d*cos(beta_val)];
    
    % Función para calcular las aceleraciones (útil para simulación)
    helpers.calc_accelerations = Function('calc_accelerations', ...
        {states, controls}, {x_ddot, beta_ddot}, ...
        {'states', 'controls'}, {'x_ddot', 'beta_ddot'});
    
    % Función para la dinámica completa (para simulación con ode45)
    f_expl = Function('f_expl', {states, controls}, {states_dot});
    helpers.f_expl = f_expl;
    
    %% ============================================
    %% INFORMACIÓN DE SALIDA
    %% ============================================
    if nargin == 0 || ~strcmp(varargin{1}, 'silent')
        fprintf('=============================================\n');
        fprintf('WHEELED BRICK 2D MODEL\n');
        fprintf('Variables:\n');
        fprintf('  x: posición del EJE de ruedas [m]\n');
        fprintf('  β: ángulo de pitch (β=0 → brick vertical) [rad]\n');
        fprintf('  τ_L, τ_R: torques en las ruedas [Nm]\n');
        fprintf('\nParámetros físicos:\n');
        fprintf('  m_b = %.2f kg, m_w = %.2f kg\n', m_b, m_w);
        fprintf('  R = %.3f m, d = l_z/2 = %.3f m\n', R, d);
        fprintf('  I_b = %.4f kg·m², I_w = %.4f kg·m²\n', I_b, I_w);
        fprintf('\nParámetros compactos:\n');
        fprintf('  M = m_b+2m_w+2I_w/R² = %.4f\n', M);
        fprintf('  C = m_b·d = %.4f\n', C);
        fprintf('  I = m_b·d²+I_b = %.4f\n', I);
        fprintf('\nEstructuras de salida:\n');
        fprintf('  model: Objeto AcadosModel (campos: x, xdot, u, f_expl_expr, etc.)\n');
        fprintf('  model_params: Parámetros del modelo\n');
        fprintf('  helpers: Funciones auxiliares para simulación\n');
        fprintf('=============================================\n');
    end
end