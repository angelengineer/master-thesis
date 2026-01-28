%% get_diff_robot_model_pitch_simple.m
% Modelo simplificado de PITCH SOLO - SIN parámetros
function [model, model_params, helpers] = get_diff_robot_model_pitch_only(varargin)

    import casadi.*
    
    %% ============================================
    %% PARÁMETROS FÍSICOS FIJOS
    %% ============================================
    m_b = 16.0;        % masa del cuerpo [kg]
    m_w = 1.0;         % masa de cada rueda [kg]
    R = 0.2;           % radio de rueda [m]
    d = 0.5;           % distancia eje -> CM cuerpo [m]
    g = 9.81;          % gravedad [m/s²]
    
    % Inercia de pitch
    ancho_brick = 0.8;
    alto_brick = 1.0;
    I_b_pitch = (1/12)*m_b*(ancho_brick^2 + alto_brick^2);
    I_w = 0.5*m_w*R^2;
    
    %% ============================================
    %% PARÁMETROS COMPACTOS
    %% ============================================
    M = m_b + 2*m_w + 2*I_w/(R^2);  % Masa efectiva
    C = m_b * d;                    % Acoplamiento cuerpo-ruedas
    I_eff_pitch = m_b * d^2 + I_b_pitch;  % Inercia efectiva pitch
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    % Estado: [β, β_dot]^T
    beta = SX.sym('beta');
    beta_dot = SX.sym('beta_dot');
    
    % Entrada: torque TOTAL (τ_L + τ_R)
    tau_total = SX.sym('tau_total');
    
    %% ============================================
    %% ECUACIÓN DINÁMICA SIMPLIFICADA
    %% ============================================
    % Asumimos:
    % 1. v es pequeño → término centrífugo despreciable
    % 2. ω es pequeño → término giroscópico despreciable
    % 3. β_dot es pequeño → término de Coriolis despreciable
    
    % Esta es la ecuación ESSENCIAL para estabilidad:
    % β_ddot ≈ (m_b*g*d*sinβ - C*a_desired*cosβ) / I_eff
    
    % Donde a_desired = τ_total/(R*M) es la aceleración lineal deseada
    % del controlador de velocidad
    
    % Simplificamos aún más para pequeñas inclinaciones:
    % sinβ ≈ β, cosβ ≈ 1
    
    % Fórmula final simplificada:
    beta_ddot = (m_b*g*d*beta - (C/(R*M))*tau_total) / I_eff_pitch;
    
    %% ============================================
    %% DINÁMICA DEL ESTADO
    %% ============================================
    states = [beta; beta_dot];
    states_dot = [beta_dot; beta_ddot];
    
    controls = tau_total;  % Solo torque total
    
    %% ============================================
    %% MODELO PARA ACADOS (SIN PARÁMETROS)
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 2, 1);
    model.u = controls;
    % ¡NO HAY model.p! → Sin parámetros
    
    model.f_expl_expr = states_dot;
    model.f_impl_expr = states_dot - model.xdot;
    model.name = 'pitch_simple_model';
    
    %% ============================================
    %% PARÁMETROS
    %% ============================================
    model_params = struct();
    model_params.m_b = m_b;
    model_params.m_w = m_w;
    model_params.R = R;
    model_params.d = d;
    model_params.g = g;
    model_params.I_eff_pitch = I_eff_pitch;
    model_params.C = C;
    model_params.M = M;
    
    % Ganancia para convertir torque en aceleración
    model_params.tau_to_accel = 1/(R*M);
    
    %% ============================================
    %% FUNCIONES AUXILIARES
    %% ============================================
    helpers = struct();
    
    % Función de dinámica (sin parámetros)
    f_expl = Function('f_expl', {states, controls}, {states_dot});
    helpers.f_expl = f_expl;
    
    % Convertir torque total + diferencia a torques individuales
    helpers.to_individual_torques = @(tau_total, tau_diff) ...
        [(tau_total - tau_diff)/2; (tau_total + tau_diff)/2];
    
    % Velocidad lineal aproximada a partir de pitch
    % v ≈ (m_b*g*d*sinβ) / (C*β_dot) para estado estacionario
    helpers.estimate_velocity = @(beta, beta_dot) ...
        (m_b*g*d*sin(beta)) ./ (C*beta_dot + 1e-6);  % Evitar división por 0
    
    fprintf('=============================================\n');
    fprintf('MODELO DE PITCH SIMPLIFICADO (SIN PARÁMETROS)\n');
    fprintf('Estados: [β, β_dot]\n');
    fprintf('Control: τ_total\n');
    fprintf('Ecuación: β_ddot = (m_b*g*d*β - K*τ_total) / I_eff\n');
    fprintf('Donde K = C/(R*M) = %.3f\n', C/(R*M));
    fprintf('=============================================\n');
end