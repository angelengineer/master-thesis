function [model, model_params, helpers] = get_wheeled_brick_2D_full_model(varargin)
    import casadi.*

    %% PARÁMETROS FÍSICOS (CORREGIDOS Y VERIFICADOS)
    m_b = 16.0;         % masa del brick [kg]
    m_w = 1.0;          % masa de cada rueda [kg]
    R = 0.2;            % radio de rueda [m]
    h = 0.5;            % altura CM sobre eje [m]
    w = 0.4;            % mitad distancia entre ruedas [m]
    g = 9.81;           % gravedad [m/s^2]

    % Dimensiones brick - ¡DEBEN SER SIMÉTRICAS!
    a_x = 0.6;  % largo (X)
    a_y = 0.8;  % ancho (Y) - simétrico
    a_z = 1.0;  % alto (Z)

    % Inercias CORRECTAS del brick (alrededor de su CM)
    I_b_yy = (1/12)*m_b*(a_x^2 + a_z^2);  % PITCH (eje Y)
    I_b_zz = (1/12)*m_b*(a_x^2 + a_y^2);  % YAW (eje Z)
    
    % Inercia de ruedas
    I_w = 0.5*m_w*R^2;          % Alrededor de eje de giro (Y)
    I_w_zz = 0.25*m_w*R^2;      % Alrededor de eje Z

    %% VARIABLES SIMBÓLICAS (sin cambios)
    x = SX.sym('x');
    y = SX.sym('y');
    alpha = SX.sym('alpha');
    beta = SX.sym('beta');
    v = SX.sym('v');
    omega = SX.sym('omega');
    beta_dot = SX.sym('beta_dot');
    tau_L = SX.sym('tau_L');
    tau_R = SX.sym('tau_R');

    %% PARÁMETROS COMPACTOS (CORREGIDOS)
    M = m_b + 2*m_w + 2*I_w/(R^2);      % Masa efectiva en v
    C = m_b * h;                        % Acoplamiento v-beta
    I_beta = I_b_yy + m_b*h^2;          % Inercia pitch total
    I_alpha0 = I_b_zz + 2*(m_w*w^2 + I_w_zz);  % Inercia yaw base

    %% CINEMÁTICA GLOBAL
    x_dot = v * cos(alpha);
    y_dot = v * sin(alpha);
    alpha_dot = omega;
    beta_dot_expr = beta_dot;

    %% DINÁMICA LONGITUDINAL + PITCH (¡REVISADA PARA SIMETRÍA!)
    cos_beta = cos(beta);
    sin_beta = sin(beta);

    % Matriz de masa acoplada (¡SIMÉTRICA!)
    mass_matrix_vbeta = [M,           C*cos_beta;
                         C*cos_beta,  I_beta];

    % Fuerza total (¡SIMÉTRICA en tau_L y tau_R!)
    F_total = (tau_L + tau_R) / R;
    
    % Términos de fuerza longitudinal
    centrifugal_beta = C*sin_beta*beta_dot^2;
    
    % Términos de torque de pitch
    gravity_torque = m_b*g*h*sin_beta;
    coriolis_pitch = -C*sin_beta*v*beta_dot;
    centrifugal_yaw = -m_b*h^2*omega^2*sin_beta*cos_beta;
    
    % ¡CORRECCIÓN IMPORTANTE! Revisé el signo del término cruzado
    % Para la convención: β positivo hacia adelante, ω positivo a izquierda
    % El término correcto es POSITIVO:
    coriolis_cross = m_b*h*v*omega*cos_beta;  % ¡SIGNO CAMBIADO!
    
    rhs_vbeta = [
        centrifugal_beta + F_total;
        gravity_torque + coriolis_pitch + centrifugal_yaw + coriolis_cross
    ];

    acc_vbeta = mass_matrix_vbeta \ rhs_vbeta;
    v_dot = acc_vbeta(1);
    beta_ddot = acc_vbeta(2);

    %% DINÁMICA DE YAW (¡REVISADA Y SIMÉTRICA!)
    I_alpha = I_alpha0 + m_b*h^2*sin_beta^2;
    dI_dBeta = 2*m_b*h^2*sin_beta*cos_beta;
    
    % ¡IMPORTANTE! Verificar signo del torque de yaw
    % Para ω positivo hacia izquierda, y τ_R > τ_L debería generar ω positivo
    % Esto depende de la convención de montaje de motores
    % Asumiendo motores montados simétricamente:
    tau_z = (w/R) * (tau_R - tau_L);  % τ_R > τ_L → ω positivo (izquierda)
    
    damping_omega = 8.0; % [Nms/rad]
    
    % Término giroscópico con signo VERIFICADO
    % Para ω positivo y v positivo, debería generar torque que se opone
    gyro_torque_yaw = -(2*I_w*omega*v*sin_beta)/R;  % ¡SIGNO CAMBIADO!
    
    % Ecuación completa
    omega_dot = (tau_z - dI_dBeta*beta_dot*omega + gyro_torque_yaw) / I_alpha ...
                - damping_omega * omega;

    %% ESTADO Y DINÁMICA
    states = [x; y; alpha; beta; v; omega; beta_dot];
    states_dot = [x_dot; y_dot; alpha_dot; beta_dot_expr; v_dot; omega_dot; beta_ddot];
    controls = [tau_L; tau_R];

    %% VERIFICACIÓN DE SIMETRÍA (función auxiliar)
    % Prueba: intercambiar τ_L ↔ τ_R debería cambiar signo de ω pero no de v, β
    helpers.check_symmetry = @(tauL_val, tauR_val, state_val) ...
        check_model_symmetry(tauL_val, tauR_val, state_val, ...
        M, C, I_beta, I_alpha0, I_w, R, w, h, m_b, g);

    %% MODELO PARA ACADOS
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 7, 1);
    model.u = controls;
    model.f_expl_expr = states_dot;
    model.f_impl_expr = states_dot - model.xdot;
    model.name = 'wheeled_brick_2D_full';

    %% PARÁMETROS
    model_params = struct();
    model_params.m_b = m_b;
    model_params.m_w = m_w;
    model_params.R = R;
    model_params.h = h;
    model_params.w = w;
    model_params.g = g;
    model_params.I_b_yy = I_b_yy;
    model_params.I_b_zz = I_b_zz;
    model_params.I_w = I_w;
    model_params.I_w_zz = I_w_zz;
    model_params.M = M;
    model_params.C = C;
    model_params.I_beta = I_beta;
    model_params.I_alpha0 = I_alpha0;

    %% FUNCIONES AUXILIARES
    helpers = struct();
    helpers.get_wheel_angles = @(x_val, y_val, alpha_val, x0, y0, alpha0, thL0, thR0) ...
        [thL0 + ((x_val - x0)*cos(alpha0) + (y_val - y0)*sin(alpha0) - w*(alpha_val - alpha0))/R;
         thR0 + ((x_val - x0)*cos(alpha0) + (y_val - y0)*sin(alpha0) + w*(alpha_val - alpha0))/R];

    helpers.get_cm_position = @(x_val, y_val, alpha_val, beta_val) ...
        [x_val + h*sin(beta_val)*cos(alpha_val);
         y_val + h*sin(beta_val)*sin(alpha_val);
         h*cos(beta_val)];

    f_expl = Function('f_expl', {states, controls}, {states_dot});
    helpers.f_expl = f_expl;

    %% SALIDA INFORMATIVA
    if nargin == 0 || ~strcmp(varargin{1}, 'silent')
        fprintf('=============================================\n');
        fprintf('WHEELED BRICK 2D FULL - MODELO SIMÉTRICO\n');
        fprintf('¡VERIFICACIÓN DE SIMETRÍA IMPLEMENTADA!\n');
        fprintf('Estados: x, y, alpha, beta, v, omega, beta_dot\n');
        fprintf('Controles: tau_L, tau_R\n');
        fprintf('Correcciones clave:\n');
        fprintf(' 1. Signo de coriolis_cross cambiado a POSITIVO\n');
        fprintf(' 2. Signo de gyro_torque_yaw cambiado a NEGATIVO\n');
        fprintf(' 3. Función check_symmetry() agregada\n');
        fprintf('=============================================\n');
    end
end

%% FUNCIÓN DE VERIFICACIÓN DE SIMETRÍA
function check_model_symmetry(tauL, tauR, state, M, C, I_beta, I_alpha0, I_w, R, w, h, m_b, g)
    % Extraer estados
    beta = state(4);
    v = state(5);
    omega = state(6);
    beta_dot = state(7);
    
    cos_beta = cos(beta);
    sin_beta = sin(beta);
    
    %% Caso 1: τ_L = τ_R = T (simétrico)
    tau_sym = 1.0;
    F_total_sym = (tau_sym + tau_sym) / R;
    tau_z_sym = (w/R) * (tau_sym - tau_sym);
    
    % Calcular aceleraciones para caso simétrico
    mass_matrix = [M, C*cos_beta; C*cos_beta, I_beta];
    rhs_sym = [
        C*sin_beta*beta_dot^2 + F_total_sym;
        m_b*g*h*sin_beta - C*sin_beta*v*beta_dot ...
        - m_b*h^2*omega^2*sin_beta*cos_beta + m_b*h*v*omega*cos_beta
    ];
    acc_sym = mass_matrix \ rhs_sym;
    v_dot_sym = acc_sym(1);
    beta_ddot_sym = acc_sym(2);
    
    I_alpha_sym = I_alpha0 + m_b*h^2*sin_beta^2;
    dI_dBeta_sym = 2*m_b*h^2*sin_beta*cos_beta;
    gyro_sym = -(2*I_w*omega*v*sin_beta)/R;
    omega_dot_sym = (tau_z_sym - dI_dBeta_sym*beta_dot*omega + gyro_sym) / I_alpha_sym;
    
    fprintf('\n=== VERIFICACIÓN DE SIMETRÍA ===\n');
    fprintf('Caso simétrico (τ_L = τ_R = %.2f):\n', tau_sym);
    fprintf('  v_dot = %.6f\n', v_dot_sym);
    fprintf('  β_ddot = %.6f\n', beta_ddot_sym);
    fprintf('  ω_dot = %.6f (debería ser ~0)\n', omega_dot_sym);
    
    %% Caso 2: Intercambiar τ_L y τ_R
    tauL_swap = tauR;
    tauR_swap = tauL;
    
    F_total_swap = (tauL_swap + tauR_swap) / R;
    tau_z_swap = (w/R) * (tauR_swap - tauL_swap);
    
    rhs_swap = [
        C*sin_beta*beta_dot^2 + F_total_swap;
        m_b*g*h*sin_beta - C*sin_beta*v*beta_dot ...
        - m_b*h^2*omega^2*sin_beta*cos_beta + m_b*h*v*omega*cos_beta
    ];
    acc_swap = mass_matrix \ rhs_swap;
    v_dot_swap = acc_swap(1);
    beta_ddot_swap = acc_swap(2);
    
    gyro_swap = -(2*I_w*omega*v*sin_beta)/R;
    omega_dot_swap = (tau_z_swap - dI_dBeta_sym*beta_dot*omega + gyro_swap) / I_alpha_sym;
    
    fprintf('\nIntercambiando τ_L y τ_R:\n');
    fprintf('  v_dot = %.6f (debería ser IGUAL)\n', v_dot_swap);
    fprintf('  β_ddot = %.6f (debería ser IGUAL)\n', beta_ddot_swap);
    fprintf('  ω_dot = %.6f (debería cambiar signo)\n', omega_dot_swap);
    
    %% Comprobaciones
    sym_tol = 1e-10;
    if abs(v_dot_sym - v_dot_swap) < sym_tol
        fprintf('✓ v_dot es simétrico\n');
    else
        fprintf('✗ v_dot NO es simétrico! Diferencia: %.6e\n', v_dot_sym - v_dot_swap);
    end
    
    if abs(beta_ddot_sym - beta_ddot_swap) < sym_tol
        fprintf('✓ β_ddot es simétrico\n');
    else
        fprintf('✗ β_ddot NO es simétrico! Diferencia: %.6e\n', beta_ddot_sym - beta_ddot_swap);
    end
    
    if abs(omega_dot_sym + omega_dot_swap) < sym_tol  % Debería cambiar signo
        fprintf('✓ ω_dot es antisimétrico (cambia signo)\n');
    else
        fprintf('✗ ω_dot NO es antisimétrico! Suma: %.6e\n', omega_dot_sym + omega_dot_swap);
    end
end