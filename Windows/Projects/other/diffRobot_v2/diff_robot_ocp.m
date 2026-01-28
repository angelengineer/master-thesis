import casadi.*
    
    check_acados_requirements();
    
    %% ================= CONFIGURACIÓN =================
    N = 30;      % Menos steps (más rápido)
    T = 1;     % Horizonte muy corto para respuesta rápida
    
    %% ================= MODELO SIMPLE =================
    [model, params, helpers] = get_diff_robot_model_pitch_only();
    nx = length(model.x);   % = 2
    nu = length(model.u);   % = 1
    
    %% ================= OCP =================
    ocp = AcadosOcp();
    ocp.model = model;
    
    %% ================= FUNCIÓN DE COSTE =================
    % Pesos ajustados para respuesta rápida
    W_x = diag([1e3, 2e3]);  % β mucho más importante que β_dot
    W_u = 1e-1;              % Penalización moderada del control
    
    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.W_0 = blkdiag(W_x, W_u);
    ocp.cost.yref_0 = zeros(nx + nu, 1);
    ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);
    
    ocp.cost.cost_type = 'NONLINEAR_LS';
    ocp.cost.W = blkdiag(W_x, W_u);
    ocp.cost.yref = zeros(nx + nu, 1);
    ocp.model.cost_y_expr = vertcat(model.x, model.u);
    
    ocp.cost.cost_type_e = 'NONLINEAR_LS';
    ocp.cost.W_e = 3.0 * W_x;
    ocp.cost.yref_e = zeros(nx, 1);
    ocp.model.cost_y_expr_e = model.x;
    
    %% ================= CONSTRAINTS - SIMPLES =================
    ocp.constraints.constr_type = 'BGH';
    
    % Límites de torque total
    tau_max_total = 20.0;  % Nm
    ocp.constraints.idxbu = 0;
    ocp.constraints.lbu = -tau_max_total;
    ocp.constraints.ubu = tau_max_total;
    
    % Límites de estados
    beta_max = deg2rad(15);      % Más restrictivo
    beta_dot_max = deg2rad(100); % Más restrictivo
    
    ocp.constraints.idxbx = [0; 1];
    ocp.constraints.lbx = [-beta_max; -beta_dot_max];
    ocp.constraints.ubx = [beta_max; beta_dot_max];
    
    % Estado inicial fijo
    ocp.constraints.idxbx_0 = [0; 1];
    ocp.constraints.lbx_0 = [-beta_max; -beta_dot_max];
    ocp.constraints.ubx_0 = [beta_max; beta_dot_max];
    
    % Sin restricciones terminales
    ocp.constraints.idxbx_e = [];
    
    %% ================= SOLVER OPTIONS =================
    ocp.solver_options.N_horizon = N;
    ocp.solver_options.tf = T;
    ocp.solver_options.nlp_solver_type = 'SQP_RTI';
    ocp.solver_options.integrator_type = 'ERK';
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
    ocp.solver_options.nlp_solver_max_iter = 3;      % ¡Solo 3 iteraciones!
    ocp.solver_options.qp_solver_iter_max = 10;
    ocp.solver_options.print_level = 0;
    ocp.solver_options.globalization = 'FIXED_STEP';
    ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';
    ocp.simulink_opts = simulink_opts;

    %% ================= CREAR SOLVER =================
    fprintf('Creando MPC simple de pitch...\n');
    ocp_solver = AcadosOcpSolver(ocp);
    
    %% ================= FUNCIONES DEL CONTROLADOR =================
    
    % Función principal de control
    function [tau_total, tau_individual] = control_pitch_simple(...
            current_beta, current_beta_dot, beta_ref, tau_diff)
        % current_beta, current_beta_dot: estados actuales
        % beta_ref: referencia de pitch (del controlador de velocidad)
        % tau_diff: diferencia de torque para giro
        
        % Estado actual
        current_state = [current_beta; current_beta_dot];
        
        % 1. Fijar estado inicial
        ocp_solver.set('constr_lbx_0', current_state);
        ocp_solver.set('constr_ubx_0', current_state);
        
        % 2. Actualizar referencia (solo en el primer paso del horizonte)
        % Referencia: [β_ref, 0] (queremos β_ref y β_dot=0)
        yref = [beta_ref; 0; 0];  % [β_ref, 0, τ_ref=0]
        ocp_solver.set('yref', 0, yref);
        
        % 3. Warm-start simple
        % Desplazar solución anterior
        x_ws = zeros(nx, N+1);
        u_ws = zeros(nu, N);
        
        for i = 0:N
            x_ws(:, i+1) = ocp_solver.get('x', i);
        end
        for i = 0:N-1
            u_ws(:, i+1) = ocp_solver.get('u', i);
        end
        
        % Desplazar
        x_ws(:, 1:end-1) = x_ws(:, 2:end);
        x_ws(:, end) = x_ws(:, end-1);
        u_ws(:, 1:end-1) = u_ws(:, 2:end);
        u_ws(:, end) = u_ws(:, end-1);
        
        % Estado inicial actual
        x_ws(:, 1) = current_state;
        
        ocp_solver.set('init_x', x_ws);
        ocp_solver.set('init_u', u_ws);
        
        % 4. Resolver (1 iteración RTI)
        ocp_solver.solve();
        
        % 5. Obtener torque total
        tau_total = ocp_solver.get('u', 0);
        
        % 6. Convertir a torques individuales
        tau_individual = helpers.to_individual_torques(tau_total, tau_diff);
        
        % 7. Limitar saturación individual
        tau_max_individual = 12.0;  % Límite por motor
        tau_individual(1) = max(min(tau_individual(1), tau_max_individual), -tau_max_individual);
        tau_individual(2) = max(min(tau_individual(2), tau_max_individual), -tau_max_individual);
    end
    
    %% ================= EMPAQUETAR CONTROLADOR =================
    mpc_controller = struct();
    mpc_controller.control_pitch = @control_pitch_simple;
    mpc_controller.ocp_solver = ocp_solver;
    mpc_controller.params = params;
    mpc_controller.helpers = helpers;
    
    fprintf('============================================\n');
    fprintf('MPC SIMPLE DE PITCH CONFIGURADO\n');
    fprintf('Sin parámetros - Computación más rápida\n');
    fprintf('Tiempo estimado por iteración: < 1 ms\n');
    fprintf('============================================\n');