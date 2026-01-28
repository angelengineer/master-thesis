function [model, model_params, helpers] = get_diff_robot_model(varargin)

    %   model: Objeto AcadosModel (solo campos permitidos)
    %   model_params: Estructura separada con parámetros
    %   helpers: Estructura con funciones auxiliares
    
    import casadi.*
    
    %% ============================================
    %% PARÁMETROS FÍSICOS
    %% ============================================
    % Valores por defecto (pueden modificarse)
    m_b = 16.0;        % masa del cuerpo (chasis) [kg]
    m_w = 1.0;         % masa de cada rueda [kg]
    R = 0.2;           % radio de rueda [m]
    d = 0.5;           % distancia eje -> CM cuerpo [m]
    g = 9.81;          % gravedad [m/s²]
    W = 0.6;           % separación entre ruedas (track width) [m]
    
    % Dimensiones del cuerpo para inercias
    ancho_brick = 0.8;     % ancho en dirección Y [m]
    alto_brick = 1.0;      % altura del cuerpo [m]
    prof_brick = 0.4;      % profundidad (dirección X) [m]
    
    % Momentos de inercia
    I_b_pitch = (1/12)*m_b*(ancho_brick^2 + alto_brick^2);  % Inercia en pitch
    I_b_yaw = (1/12)*m_b*(ancho_brick^2 + prof_brick^2);    % Inercia en yaw
    I_w = 0.5*m_w*R^2;     % Inercia rotacional de cada rueda
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    % Estado como se definió: [β, β_dot, v, ω]^T
    beta = SX.sym('beta');      % ángulo de pitch (β)
    beta_dot = SX.sym('beta_dot');  % velocidad angular de pitch
    v = SX.sym('v');            % velocidad lineal hacia adelante
    omega = SX.sym('omega');    % velocidad angular de yaw
    
    % Entradas: torques de las ruedas
    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');
    
    %% ============================================
    %% PARÁMETROS COMPACTOS
    %% ============================================
    % Masa efectiva total para movimiento longitudinal
    M = m_b + 2*m_w + 2*I_w/(R^2);
    
    % Acoplamiento cuerpo-ruedas
    C = m_b * d;
    
    % Inercia efectiva para pitch
    I_eff_pitch = m_b * d^2 + I_b_pitch;
    
    % Inercia efectiva para yaw
    I_eff_yaw = m_b * d^2 * sin(beta)^2 + I_b_yaw + (W^2/(2*R^2))*I_w;
    
    % Relaciones geométricas
    cos_beta = cos(beta);
    sin_beta = sin(beta);
    
    %% ============================================
    %% ECUACIONES DINÁMICAS (LAGRANGIANA)
    %% ============================================
    % 1. Aceleración longitudinal (v_dot)
    % Fuerzas: gravedad (debido a la inclinación), torques de ruedas
    v_dot_num = (tau_L + tau_R)/R - C*beta_dot^2*sin_beta - m_b*g*sin_beta*cos_beta;
    v_dot = v_dot_num / M;
    
    % 2. Aceleración angular de pitch (beta_ddot)
    % Momento: gravedad, fuerza centrífuga, acoplamiento con aceleración lineal
    beta_ddot_num = m_b*g*d*sin_beta - C*v_dot*cos_beta - C*omega^2*d*sin_beta*cos_beta;
    beta_ddot = beta_ddot_num / I_eff_pitch;
    
    % 3. Aceleración angular de yaw (omega_dot)
    % Momento de yaw: diferencia de torques, efectos giroscópicos
    tau_diff = tau_R - tau_L;
    omega_dot_num = (W/(2*R))*tau_diff - 2*m_b*d^2*beta_dot*omega*sin_beta*cos_beta;
    omega_dot = omega_dot_num / I_eff_yaw;
    
    %% ============================================
    %% DINÁMICA COMPLETA DEL ESTADO
    %% ============================================
    % Vector de estado
    states = [beta; beta_dot; v; omega];
    
    % Derivadas del estado
    states_dot = [beta_dot; beta_ddot; v_dot; omega_dot];
    
    % Entradas
    controls = [tau_L; tau_R];
    
    %% ============================================
    %% MODELO PARA ACADOS
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 4, 1);
    model.u = controls;
    
    % Formulación explícita
    model.f_expl_expr = states_dot;
    
    % Formulación implícita
    model.f_impl_expr = states_dot - model.xdot;
    
    model.name = 'diff_robot_model';
    
    %% ============================================
    %% PARÁMETROS EN ESTRUCTURA SEPARADA
    %% ============================================
    model_params = struct();
    model_params.m_b = m_b;
    model_params.m_w = m_w;
    model_params.R = R;
    model_params.d = d;
    model_params.g = g;
    model_params.W = W;
    model_params.I_b_pitch = I_b_pitch;
    model_params.I_b_yaw = I_b_yaw;
    model_params.I_w = I_w;
    model_params.M = M;
    model_params.C = C;
    model_params.I_eff_pitch = I_eff_pitch;
    model_params.I_eff_yaw = I_eff_yaw;
    
    %% ============================================
    %% FUNCIONES AUXILIARES
    %% ============================================
    helpers = struct();
    
    % 1. Cinemática de las ruedas (a partir de v y ω)
    helpers.get_wheel_velocities = @(v_val, omega_val) ...
        [(v_val/R) - (W/(2*R))*omega_val;   % φ_dot_L
         (v_val/R) + (W/(2*R))*omega_val];  % φ_dot_R
    
    % 2. Posición del centro de masa del cuerpo
    helpers.get_body_cm_position = @(beta_val) ...
        [d*sin(beta_val);      % x relativo al eje
         0;                    % y (cero por simetría)
         d*cos(beta_val)];     % z (altura)
    
    % 3. Velocidades de las ruedas para simulación
    helpers.get_wheel_speeds = Function('get_wheel_speeds', ...
        {states}, {helpers.get_wheel_velocities(v, omega)}, ...
        {'states'}, {'phi_dot_LR'});
    
    % 4. Función para la dinámica completa (para simulación)
    f_expl = Function('f_expl', {states, controls}, {states_dot});
    helpers.f_expl = f_expl;
    
    % 5. Función para calcular aceleraciones individuales
    helpers.calc_accelerations = Function('calc_accelerations', ...
        {states, controls}, {beta_ddot, v_dot, omega_dot}, ...
        {'states', 'controls'}, {'beta_ddot', 'v_dot', 'omega_dot'});
    
    % 6. Transformación a coordenadas globales (para trayectorias)
    % Asume que el robot parte de (0,0,0)
    helpers.to_global_pose = @(x_pos, y_pos, theta, beta_val) ...
        [x_pos + d*sin(beta_val)*cos(theta);
         y_pos + d*sin(beta_val)*sin(theta);
         theta;
         R + d*cos(beta_val)];  % altura sobre el suelo
    
    %% ============================================
    %% VALIDACIÓN DE PARÁMETROS
    %% ============================================
    % Condición para auto-balanceo: CM debe estar sobre el eje
    if d <= 0
        warning('La distancia d debe ser positiva para un robot auto-balanceado');
    end
    
    % Condición para estabilidad estática
    static_margin = m_b*g*d;
    if static_margin < 5.0  % Umbral arbitrario
        warning('El robot puede tener dificultades para auto-balancearse');
    end
    
    %% ============================================
    %% INFORMACIÓN DE SALIDA
    %% ============================================
    if nargin == 0 || ~strcmp(varargin{1}, 'silent')
        fprintf('=============================================\n');
        fprintf('SEGWAY 2D NONLINEAR MODEL\n');
        fprintf('State variables:\n');
        fprintf('  β: pitch angle (β=0 → vertical) [rad]\n');
        fprintf('  β_dot: pitch angular velocity [rad/s]\n');
        fprintf('  v: forward linear velocity [m/s]\n');
        fprintf('  ω: yaw angular velocity [rad/s]\n');
        fprintf('\nControl inputs:\n');
        fprintf('  τ_L, τ_R: wheel torques [Nm]\n');
        fprintf('\nPhysical parameters:\n');
        fprintf('  m_b = %.2f kg, m_w = %.2f kg\n', m_b, m_w);
        fprintf('  R = %.3f m, d = %.3f m, W = %.3f m\n', R, d, W);
        fprintf('  I_b_pitch = %.4f kg·m²\n', I_b_pitch);
        fprintf('  I_b_yaw = %.4f kg·m²\n', I_b_yaw);
        fprintf('  I_w = %.4f kg·m²\n', I_w);
        fprintf('\nCompact parameters:\n');
        fprintf('  M = m_b+2m_w+2I_w/R² = %.4f\n', M);
        fprintf('  C = m_b·d = %.4f\n', C);
        fprintf('  I_eff_pitch = m_b·d²+I_b_pitch = %.4f\n', I_eff_pitch);
        fprintf('  I_eff_yaw = m_b·d²·sin²β + I_b_yaw + (W²/2R²)I_w\n');
        fprintf('\nModel characteristics:\n');
        fprintf('  • Full nonlinear (no small-angle assumption)\n');
        fprintf('  • Coupled pitch-longitudinal dynamics\n');
        fprintf('  • Independent yaw control via torque differential\n');
        fprintf('  • Suitable for MPC (control-oriented, low-order)\n');
        fprintf('=============================================\n');
    end
end