function model = get_wheeled_robot_model(varargin)
    import casadi.*
    
    %% ============================================
    %% PARÁMETROS FÍSICOS
    %% ============================================
    
    % Masas [kg]
    m_w = 1.0;      % masa de cada rueda
    m_b = 16.0;     % masa del cuerpo principal
    m_m = 5.0;      % masa del mecanismo de elevación
    m_a = 5.0;      % masa del motor del brazo
    m_f = 2.0;      % masa de la horquilla (fork)
    
    % Momentos de inercia [kg·m²]
    I_w = 0.02;     % inercia de cada rueda
    I_b = 1.4533;   % inercia del cuerpo principal
    I_m = 0.9417;   % inercia del mecanismo de elevación
    I_a = 0.1531;   % inercia del motor del brazo
    I_f = 0.03;     % inercia de la horquilla
    
    % Parámetros geométricos [m]
    r_w = 0.2;      % radio de rueda
    
    % Centros de gravedad en frame {B}
    l_bx = 0.0;
    l_bz = 0.5;
    
    l_mx = -0.15;
    l_mz = 1.2;
    
    l_ax = -0.1;
    l_az = 1.05;
    
    l_fx = 0.3;
    
    % Gravedad
    g = 9.81;
    
    %% ============================================
    %% VARIABLES SIMBÓLICAS DE ESTADO
    %% ============================================
    
    x = SX.sym('x');
    theta_p = SX.sym('theta_p');
    d_m = SX.sym('d_m');
    theta_a = SX.sym('theta_a');
    
    dx = SX.sym('dx');
    dtheta_p = SX.sym('dtheta_p');
    dd_m = SX.sym('dd_m');
    dtheta_a = SX.sym('dtheta_a');
    
    %% ============================================
    %% VARIABLES DE CONTROL (SOLO 3 ACTUADORES)
    %% ============================================
    
    tau_x = SX.sym('tau_x');   % ruedas
    tau_m = SX.sym('tau_m');   % lift
    tau_a = SX.sym('tau_a');   % fork
    
    % Vector de control REAL (3 elementos)
    controls = [tau_x; tau_m; tau_a];
    
    %% ============================================
    %% ENERGÍAS (igual que antes, no cambia la dinámica pasiva)
    %% ============================================
    
    % --- Ruedas ---
    T_w = 0.5 * m_w * dx^2 + 0.5 * I_w * (dx/r_w)^2;
    
    % --- Cuerpo ---
    x_b = x + l_bx*cos(theta_p) + l_bz*sin(theta_p);
    z_b = r_w - l_bx*sin(theta_p) + l_bz*cos(theta_p);
    dx_b = dx - l_bx*dtheta_p*sin(theta_p) + l_bz*dtheta_p*cos(theta_p);
    dz_b = -l_bx*dtheta_p*cos(theta_p) - l_bz*dtheta_p*sin(theta_p);
    T_b = 0.5 * m_b * (dx_b^2 + dz_b^2) + 0.5 * I_b * dtheta_p^2;
    U_b = m_b * g * z_b;
    
    % --- Mecanismo de elevación ---
    x_m = x + l_mx*cos(theta_p) + l_mz*sin(theta_p);
    z_m = r_w - l_mx*sin(theta_p) + l_mz*cos(theta_p);
    dx_m = dx - l_mx*dtheta_p*sin(theta_p) + l_mz*dtheta_p*cos(theta_p);
    dz_m = -l_mx*dtheta_p*cos(theta_p) - l_mz*dtheta_p*sin(theta_p);
    T_m = 0.5 * m_m * (dx_m^2 + dz_m^2) + 0.5 * I_m * dtheta_p^2;
    U_m = m_m * g * z_m;
    
    % --- Motor del brazo ---
    x_a = x + l_ax*cos(theta_p) + (l_az + d_m)*sin(theta_p);
    z_a = r_w - l_ax*sin(theta_p) + (l_az + d_m)*cos(theta_p);
    dx_a = dx - l_ax*dtheta_p*sin(theta_p) + (l_az + d_m)*dtheta_p*cos(theta_p) + dd_m*sin(theta_p);
    dz_a = -l_ax*dtheta_p*cos(theta_p) - (l_az + d_m)*dtheta_p*sin(theta_p) + dd_m*cos(theta_p);
    T_a = 0.5 * m_a * (dx_a^2 + dz_a^2) + 0.5 * I_a * dtheta_p^2;
    U_a = m_a * g * z_a;
    
    % --- Horquilla ---
    x_f = x + l_ax*cos(theta_p) + (l_az + d_m)*sin(theta_p) + l_fx * cos(theta_p + theta_a);
    z_f = r_w - l_ax*sin(theta_p) + (l_az + d_m)*cos(theta_p) - l_fx * sin(theta_p + theta_a);
    dx_f = dx - l_ax*dtheta_p*sin(theta_p) + (l_az + d_m)*dtheta_p*cos(theta_p) + dd_m*sin(theta_p) - ...
           l_fx * sin(theta_p + theta_a)*(dtheta_p + dtheta_a);
    dz_f = -l_ax*dtheta_p*cos(theta_p) - (l_az + d_m)*dtheta_p*sin(theta_p) + dd_m*cos(theta_p) - ...
           l_fx * cos(theta_p + theta_a)*(dtheta_p + dtheta_a);
    T_f = 0.5 * m_f * (dx_f^2 + dz_f^2) + 0.5 * I_f * (dtheta_p + dtheta_a)^2;
    U_f = m_f * g * z_f;
    
    %% ============================================
    %% LAGRANGIANO
    %% ============================================
    
    T_total = 2*T_w + T_b + T_m + T_a + T_f;
    U_total = U_b + U_m + U_a + U_f;
    L = T_total - U_total;
    
    %% ============================================
    %% MATRICES DINÁMICAS
    %% ============================================
    
    q = [x; theta_p; d_m; theta_a];
    dq = [dx; dtheta_p; dd_m; dtheta_a];
    n = length(q);
    
    % Matriz de inercia M(q)
    M = SX.zeros(n, n);
    for i = 1:n
        for j = 1:n
            M(i,j) = jacobian(jacobian(L, dq(j)), dq(i));
        end
    end
    
    % Matriz de Coriolis C(q,dq)
    C = SX.zeros(n, n);
    for k = 1:n
        for j = 1:n
            c_kj = 0;
            for i = 1:n
                c_kj = c_kj + 0.5 * (jacobian(M(k,j), q(i)) + ...
                                      jacobian(M(k,i), q(j)) - ...
                                      jacobian(M(i,j), q(k))) * dq(i);
            end
            C(k,j) = c_kj;
        end
    end
    
    % Vector de gravedad G(q)
    G = SX.zeros(n, 1);
    for i = 1:n
        G(i) = jacobian(U_total, q(i));
    end
    
    %% ============================================
    %% ECUACIONES DE MOVIMIENTO (SIN ACTUACIÓN EN PITCH)
    %% ============================================
    
    % Vector de torques/fuerzas generalizadas: [tau_x, 0, tau_m, tau_a]
    tau_full = [tau_x; 0; tau_m; tau_a];  % ¡tau_p = 0! No hay actuador en pitch
    
    % Resolver: M*ddq = tau_full - C*dq - G
    ddq = inv(M) * (tau_full - C*dq - G);
    
    ddx = ddq(1);
    ddtheta_p = ddq(2);
    ddd_m = ddq(3);
    ddtheta_a = ddq(4);
    
    %% ============================================
    %% ESTADO Y DINÁMICA
    %% ============================================
    
    states = [x; theta_p; d_m; theta_a; 
              dx; dtheta_p; dd_m; dtheta_a];
    states_dot = [dx; dtheta_p; dd_m; dtheta_a;
                  ddx; ddtheta_p; ddd_m; ddtheta_a];
    
    %% ============================================
    %% MODELO PARA ACADOS
    %% ============================================
    
    model = AcadosModel();
    model.x = states;
    model.xdot = SX.sym('xdot', 8, 1);
    model.u = controls;  % ¡SOLO 3 controles!
    model.f_expl_expr = states_dot;
    model.f_impl_expr = model.xdot - states_dot;
    model.name = 'wheeled_robot_lift_fork';
    
    fprintf('Robot model created: %s\n', model.name);
    fprintf('  States: %d\n', length(states));
    fprintf('  Controls: %d (tau_x, tau_m, tau_a only)\n', length(controls));
end