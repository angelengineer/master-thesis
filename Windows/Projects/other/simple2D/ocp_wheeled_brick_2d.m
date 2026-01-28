import casadi.*
check_acados_requirements()

%% 1. CONFIGURACIÓN
N = 50;    
T = 2.0;   
x0 = [0; 0; 0; 0; 0; 0; 0]; 

%% 2. MODELO
[model, params, helpers] = get_wheeled_brick_model_2d();
nx = length(model.x); 
nu = length(model.u); 

ocp = AcadosOcp();
ocp.model = model;

%% 3. COSTOS
% Referencias de velocidad (v y omega)
TARGET_V = 1.0;
TARGET_W = 0.5;

W_x_diag = [1e-9, 1e-9, 1e-9, 50, 1e2, 1e2, 1e0]; 
W_x = diag(W_x_diag);
W_u = 1e-1 * eye(nu);

ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.W_0 = blkdiag(W_x, W_u);

yref_target = [0; 0; 0; 0; TARGET_V; TARGET_W; 0; 0; 0];
ocp.cost.yref = yref_target;
ocp.cost.yref_0 = yref_target;
ocp.model.cost_y_expr = vertcat(model.x, model.u);
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% ==========================================================
% AGREGAR SLACK VARIABLES (SOFT CONSTRAINTS) AL PITCH
% ==========================================================
% ==========================================================
% AGREGAR SLACK VARIABLES (SOFT CONSTRAINTS) AL PITCH
% ==========================================================
% 1. Decimos que la restricción en idxbx(1) (que es el Pitch) es blanda
ocp.constraints.idxsbx = 0; % Index relativo a idxbx (0 es el primer elemento de idxbx)

% 2. Pesos de la multa para el PATH (Etapas 1 a N-1)
ocp.cost.Zl = 1e4 * ones(1,1); 
ocp.cost.Zu = 1e4 * ones(1,1); 
ocp.cost.zl = 1e3 * ones(1,1); 
ocp.cost.zu = 1e3 * ones(1,1);

% 3. Pesos de la multa para la etapa TERMINAL (Etapa N)
% Es vital definirlos también para N, si no, el solver puede fallar al final
ocp.constraints.idxsbx_e = 0; 
ocp.cost.Zl_e = 1e4 * ones(1,1); 
ocp.cost.Zu_e = 1e4 * ones(1,1); 
ocp.cost.zl_e = 1e3 * ones(1,1); 
ocp.cost.zu_e = 1e3 * ones(1,1);

%% 4. RESTRICCIONES
U_max = 12; 
ocp.constraints.constr_type = 'BGH';

% Restricciones de Control (Duras - Hard)
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -U_max * ones(nu,1);
ocp.constraints.ubu =  U_max * ones(nu,1);

% Restricciones de Estado
% Para usar Slacks, primero definimos la restricción como "Hard" y Acados la ablanda
beta_lim = deg2rad(30); % Límite deseado (un poco más estricto ahora que es blando)

% Vamos a definir solo beta como restricción con slack por ahora para no complicar
ocp.constraints.idxbx = 3; % Índice del Pitch (beta)
ocp.constraints.lbx = -beta_lim;
ocp.constraints.ubx =  beta_lim;

% Estado inicial fijo
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;
ocp.constraints.idxbx_0 = 0:nx-1;

% --- RESTRICCIONES ETAPA TERMINAL (N) ---
% Debemos decirle al solver que en la última etapa también limite el Pitch
ocp.constraints.idxbx_e = 3;         % El índice del estado beta
ocp.constraints.lbx_e = -beta_lim;   % Límite inferior
ocp.constraints.ubx_e =  beta_lim;   % Límite superior

%% 5. OPCIONES SOLVER
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.print_level = 0;
ocp.simulink_opts = simulink_opts;

ocp_solver = AcadosOcpSolver(ocp);