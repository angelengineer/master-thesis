import casadi.*
    
    check_acados_requirements()
    
    %% Horizonte
    N = 20;
    T = 2.0;
    x0 = [0; 0];
    
    %% Modelo con perturbación
    model = get_arm_model_with_disturbance();
    
    %% OCP
    ocp = AcadosOcp();
    ocp.model = model;
    
    %% Costo
    W_x = diag([5e3, 5e1]);
    W_u = 1e-2;
    
    theta_ref = pi/4;
    theta_dot_ref = 0.0;
    yref = [theta_ref; theta_dot_ref; 0];
    
    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.W_0 = blkdiag(W_x, W_u);
    ocp.cost.yref_0 = yref;
    ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);
    
    ocp.cost.cost_type = 'NONLINEAR_LS';
    ocp.cost.W = blkdiag(W_x, W_u);
    ocp.cost.yref = yref;
    ocp.model.cost_y_expr = vertcat(model.x, model.u);
    
    ocp.cost.cost_type_e = 'NONLINEAR_LS';
    ocp.cost.W_e = 10 * W_x;
    ocp.cost.yref_e = [theta_ref; theta_dot_ref];
    ocp.model.cost_y_expr_e = model.x;
    
    %% Restricciones
    tau_max = 20.0;
    tau_min = -20.0;
    
    ocp.constraints.constr_type = 'BGH';
    ocp.constraints.idxbu = 0;
    ocp.constraints.lbu = tau_min;
    ocp.constraints.ubu = tau_max;
    
    ocp.constraints.idxbx = 0:1;
    ocp.constraints.lbx = [-pi; -5.0];
    ocp.constraints.ubx = [pi; 5.0];
    
    ocp.constraints.idxbx_e = 0:1;
    ocp.constraints.lbx_e = [-pi; -5.0];
    ocp.constraints.ubx_e = [pi; 5.0];
    
    ocp.constraints.idxbx_0 = 0:1;
    ocp.constraints.lbx_0 = x0;
    ocp.constraints.ubx_0 = x0;
    
    %% Solver options
    ocp.solver_options.N_horizon = N;
    ocp.solver_options.tf = T;
    ocp.solver_options.nlp_solver_type = 'SQP';
    ocp.solver_options.nlp_solver_max_iter = 100;
    ocp.solver_options.integrator_type = 'IRK';
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
    ocp.solver_options.print_level = 0;
    
    %% Perturbación inicial (será actualizada por ESO)
    ocp.parameter_values = 0.0;  % d inicial
    ocp.simulink_opts = simulink_opts;

    %% Crear solver
    ocp.code_export_directory = 'c_generated_code_arm_eso';
    ocp_solver = AcadosOcpSolver(ocp);
    
    fprintf('✅ MPC con ESO creado\n');