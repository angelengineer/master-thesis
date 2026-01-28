function model = get_arm_model_with_disturbance()
    % Modelo simplificado con perturbación medida/estimada
    % Estados:   [theta; theta_dot]
    % Control:   tau
    % Parámetros: d (perturbación agregada)
    
    import casadi.*
    
    %% Parámetro nominal fijo (NO se estima)
    b0 = 1.0;  % Ganancia nominal [1/(kg·m²)]
                % Para m=5kg, r=0.4m → b0 ≈ 1.25
                % Usar un valor conservador tipo 1.0
    
    %% Estados
    theta     = SX.sym('theta');
    theta_dot = SX.sym('theta_dot');
    states = vertcat(theta, theta_dot);
    
    %% Control
    tau = SX.sym('tau');
    controls = tau;
    
    %% Parámetro: PERTURBACIÓN AGREGADA
    d = SX.sym('d');  % [rad/s²] - estimada por ESO
    params = d;
    
    %% Dinámica simplificada
    % θ̈ = b₀·τ + d(t)
    % donde d(t) incluye: gravedad + errores de modelo + cargas externas
    theta_ddot = b0 * tau + d;
    
    states_dot = vertcat(theta_dot, theta_ddot);
    
    %% Modelo Acados
    model = AcadosModel();
    model.x = states;
    model.u = controls;
    model.p = params;
    model.xdot = SX.sym('xdot', 2, 1);
    
    model.f_expl_expr = states_dot;
    model.f_impl_expr = model.xdot - states_dot;
    model.name = 'arm_model_with_disturbance';
end