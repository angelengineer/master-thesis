function [model, model_params] = get_wheeled_brick_model_2D()
    % Modelo dinámico completo 2D para robot differential drive con balanceo
    % Parámetros físicos exactos proporcionados:
    %   Cuerpo: lx=0.3m, ly=0.8m, lz=1.0m, M=16kg
    %   Ruedas: m_w=1kg, r=0.2m, t=0.05m, d=t/2+ly=0.825m
    
    import casadi.*
    
    %% ============================================
    %% PARÁMETROS FÍSICOS (VALORES EXACTOS)
    %% ============================================
    % Cuerpo (prisma rectangular)
    lx = 0.3;   % [m] ancho en dirección X (longitudinal)
    ly = 0.8;   % [m] ancho en dirección Y (lateral)
    lz = 1.0;   % [m] altura
    M_body = 16; % [kg] masa del cuerpo
    
    % Ruedas
    m_w = 1.0;  % [kg] masa de cada rueda
    r = 0.2;    % [m] radio de rueda
    t = 0.05;   % [m] espesor de rueda
    d = t/2 + ly; % [m] distancia desde eje de ruedas hasta CM del cuerpo = 0.825m
    
    % Parámetros derivados
    g = 9.81;   % [m/s²] gravedad
    
    % === MOMENTOS DE INERCIA (SIN APROXIMACIONES) ===
    % Cuerpo alrededor de su CM
    I_body_xx = (1/12)*M_body*(ly^2 + lz^2);  % Eje X (lateral)
    I_body_yy = (1/12)*M_body*(lx^2 + lz^2);  % Eje Y (longitudinal)
    I_body_zz = (1/12)*M_body*(lx^2 + ly^2);  % Eje Z (vertical)
    
    % Ruedas (cilindros sólidos)
    I_wheel_z = 0.5*m_w*r^2;                  % Alrededor eje de rotación (Z)
    I_wheel_x = (1/12)*m_w*(3*r^2 + t^2); % Alrededor ejes X/Y
    I_wheel_y = I_wheel_x;
    % Distancia entre ruedas (ajustable según diseño físico)
    L = ly + 2*r;  % [m] distancia entre centros de ruedas = 0.8 + 0.4 = 1.2m
    
    % === PARÁMETROS COMPACTOS PARA DINÁMICA ===
    % Masa total efectiva en dirección longitudinal
    M_total = M_body + 2*m_w + 2*I_wheel_z/(r^2);
    
    % Parámetros para acoplamiento pitch-longitudinal
    C_beta = M_body * d;  % Acoplamiento masa-pitch
    I_beta = M_body*d^2 + I_body_yy; % Inercia pitch efectiva
    
    % Parámetros para dinámica yaw
    I_zz_total = I_body_zz + 2*(I_wheel_z + m_w*(L/2)^2); % Inercia yaw total
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS (ESTADO COMPLETO 2D)
    %% ============================================
    % Posición y orientación global
    x = SX.sym('x');        % Posición X del punto medio entre ruedas
    y = SX.sym('y');        % Posición Y del punto medio entre ruedas
    theta = SX.sym('theta');% Orientación global (yaw)
    
    % Ángulos de configuración
    beta = SX.sym('beta');  % Ángulo pitch (0=vertical, + hacia adelante)
    
    % Velocidades
    v = SX.sym('v');        % Velocidad lineal longitudinal
    w = SX.sym('w');        % Velocidad angular (yaw rate)
    beta_dot = SX.sym('beta_dot'); % Velocidad angular pitch
    
    % Controles (entradas físicas)
    tau_L = SX.sym('tau_L');% Torque rueda izquierda
    tau_R = SX.sym('tau_R');% Torque rueda derecha
    
    %% ============================================
    %% CINEMÁTICA GLOBAL (NO HOLONÓMICA)
    %% ============================================
    % Ecuaciones cinemáticas exactas
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = w;
    
    %% ============================================
    %% DINÁMICA LONGITUDINAL + PITCH (ACOPLADO)
    %% ============================================
    % Variables trigonométricas
    cos_beta = cos(beta);
    sin_beta = sin(beta);
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    
    % Matriz de masa para subsistema [v; beta_ddot]
    mass_matrix_vbeta = [M_total,        C_beta*cos_beta;
                        C_beta*cos_beta, I_beta];
    
    % Términos de Coriolis y centrífugos
    coriolis_v = C_beta * beta_dot^2 * sin_beta; % Término no lineal pitch->long
    
    % Fuerza total longitudinal (de torques ruedas)
    F_total = (tau_L + tau_R) / r;
    
    % Momento gravitatorio en pitch
    torque_gravity = M_body * g * d * sin_beta;
    
    % Vector de fuerzas generalizadas
    rhs_vbeta = [coriolis_v + F_total;
                torque_gravity];
    
    % Resolver aceleraciones [v_dot; beta_ddot]
    accel_vbeta = mass_matrix_vbeta \ rhs_vbeta;
    v_dot = accel_vbeta(1);
    beta_ddot = accel_vbeta(2);
    
    %% ============================================
    %% DINÁMICA YAW (CON ACOPLAMIENTO GYROSCÓPICO)
    %% ============================================
    % Momento de giro neto
    M_yaw = (tau_R - tau_L) * (L/(2*r));
    
    % === TÉRMINOS GIROSCÓPICOS CRÍTICOS ===
    % 1. Acoplamiento pitch->yaw por rotación de ruedas
    gyro_wheels = (2*I_wheel_z/r) * v * sin_beta * w;
    
    % 2. Acoplamiento body pitch->yaw (efecto giroscópico del cuerpo inclinado)
    gyro_body = (I_body_zz - I_body_yy) * w * beta_dot * sin_beta;
    
    % Ecuación yaw completa (2ª ley de Newton para rotación)
    w_dot = (M_yaw - gyro_wheels - gyro_body) / I_zz_total;
    
    %% ============================================
    %% ESTADO COMPLETO Y DINÁMICA
    %% ============================================
    % Vector de estados (7 dimensiones)
    states = [x; y; theta; beta; v; w; beta_dot];
    
    % Derivadas de estados
    states_dot = [x_dot;
                  y_dot;
                  theta_dot;
                  beta_dot;
                  v_dot;
                  w_dot;
                  beta_ddot];
    
    % Vector de controles
    controls = [tau_L; tau_R];
    
    %% ============================================
    %% MODELO PARA ACADOS (SIN SIMPLIFICACIONES)
    %% ============================================
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 7, 1);
    model.u = controls;
    
    % Formulación explícita (recomendada para sistemas no lineales)
    model.f_expl_expr = states_dot;
    
    % Formulación implícita (requerida por algunos solvers)
    model.f_impl_expr = states_dot - model.xdot;
    
    model.name = 'wheeled_brick_2D_full';
    
    %% ============================================
    %% PARÁMETROS EN ESTRUCTURA SEPARADA
    %% ============================================
    model_params = struct();
    % Parámetros geométricos
    model_params.lx = lx;
    model_params.ly = ly;
    model_params.lz = lz;
    model_params.r = r;
    model_params.t = t;
    model_params.d = d;
    model_params.L = L;  % Distancia entre ruedas
    
    % Masas e inercias
    model_params.M_body = M_body;
    model_params.m_w = m_w;
    model_params.M_total = M_total;
    model_params.I_body_xx = I_body_xx;
    model_params.I_body_yy = I_body_yy;
    model_params.I_body_zz = I_body_zz;
    model_params.I_wheel_x = I_wheel_x;
    model_params.I_wheel_y = I_wheel_y;
    model_params.I_wheel_z = I_wheel_z;
    model_params.I_beta = I_beta;
    model_params.I_zz_total = I_zz_total;
    
    % Parámetros dinámicos compactos
    model_params.C_beta = C_beta;
    model_params.g = g;
    
   