import casadi.*
% 
% % options needed for the Simulink example
% if ~exist('simulink_opts','var')
%     disp('using empty simulink_opts to generate solver without simulink block')
%     simulink_opts = [];
% end
% 
% check_acados_requirements()

%% solver settings
N = 40; % number of discretization steps
T = 1; % [s] prediction horizon length
% Estado inicial: [x, β, ẋ, β̇]
x0 = [0; 0; 0; 0]; 

%% model dynamics
model = model_simplified();
nx = length(model.x); % state size = 4
nu = length(model.u); % input size = 2
np = length(model.p);   

%%Parameters

m_load_val = 5.0;          % kg
I_b_val    = 1.25;         % kg·m^2
l_bx_val   = -0.0232;      % m
l_bz_val   = 0.8750;       % m

p_val = [
    m_load_val;
    I_b_val;
    l_bx_val;
    l_bz_val
];

%% OCP formulation object
ocp = AcadosOcp();
ocp.model = model; 

ocp.parameter_values = p_val;


%% cost formulation - USING NONLINEAR_LS (simpler and more flexible)
% Pesos para estados: [x, β, ẋ, β̇]
W_x = diag([1e-8, 1e3, 1e2, 1e3]);  
%W_x = diag([1e2, 1e3, 1e2, 1e3]);  
W_x = diag([1e1, 1e2, 1e2, 1e3]);  
W_x = diag([1e-9, 1e1, 1e2, 1e3]);  
% Pesos para controles: [τ_L, τ_R]
W_u = 1e-1 * eye(nu);

% ========== COSTO INICIAL (stage 0) ==========
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);  % Peso para estados y controles
ocp.cost.yref_0 = zeros(nx+nu, 1);  % Referencia: [x_ref, β_ref, ẋ_ref, β̇_ref, τ_L_ref, τ_R_ref]
% Expresión del costo: [model.x; model.u]
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% ========== COSTO PATH (stages 1 to N-1) ==========
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
% Referencia: queremos llevar el brick a posición vertical (β=0)
% y a x=0 (posición horizontal de referencia)
ocp.cost.yref = [0; 0; 0; 0; zeros(nu,1)];
% Expresión del costo: [model.x; model.u]
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% % ========== COSTO TERMINAL (stage N) ==========
% ocp.cost.cost_type_e = 'NONLINEAR_LS';
% % Peso terminal mayor para asegurar estabilización al final
% W_e = 10 * W_x;  % 10x más peso en el costo terminal
% ocp.cost.W_e = W_e;
% ocp.cost.yref_e = zeros(nx, 1);  % Solo estados en costo terminal
% % Expresión del costo terminal: solo estados
% ocp.model.cost_y_expr_e = model.x;

%% constraints
% Límites de control (torques realistas)
U_max = 10;  % Nm

% Usar restricciones de caja
ocp.constraints.constr_type = 'BGH';

% Restricciones de CONTROL
ocp.constraints.idxbu = 0:nu-1;  % [0, 1] para 2 controles
ocp.constraints.lbu = -U_max * ones(nu,1);
ocp.constraints.ubu = U_max * ones(nu,1);

% Límites de estado
x_max = 5;
beta_max = deg2rad(45);
x_dot_max = 3;
beta_dot_max = deg2rad(180);

% Para etapas de trayectoria (1 a N-1)
ocp.constraints.idxbx = 0:nx-1;  % [0, 1, 2, 3] para 4 estados
ocp.constraints.lbx = [-x_max; -beta_max; -x_dot_max; -beta_dot_max];
ocp.constraints.ubx = [x_max; beta_max; x_dot_max; beta_dot_max];

% Para etapa terminal (N)
% ocp.constraints.idxbx_e = 0:nx-1;
% ocp.constraints.lbx_e = [-x_max; -beta_max; -x_dot_max; -beta_dot_max];
% ocp.constraints.ubx_e = [x_max; beta_max; x_dot_max; beta_dot_max];

% Para etapa inicial (0) - fijar el estado inicial
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% solver options
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP_RTI'; %SQP
ocp.solver_options.integrator_type = 'GNSF'; %ERK
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.qp_solver_warm_start = 2;
ocp.solver_options.nlp_solver_max_iter = 10; %100
ocp.solver_options.qp_solver_iter_max = 20; %50
ocp.solver_options.print_level = 1;
ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';

ocp.simulink_opts = simulink_opts;

ocp.code_export_directory = ['c_generated_code_' model.name];

%% create solver
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('Solver creado exitosamente\n');
catch ME
    fprintf('Error creando solver: %s\n', ME.message);
    rethrow(ME);
end