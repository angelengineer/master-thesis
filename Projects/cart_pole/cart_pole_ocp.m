
import casadi.*

% options needed for the Simulink example
if ~exist('simulink_opts','var')
    % disp('using acados simulink default options')
    % simulink_opts = get_acados_simulink_opts;
    disp('using empty simulink_opts to generate solver without simulink block')
    simulink_opts = [];
end

%
check_acados_requirements()

%% solver settings
N = 40; % number of discretization steps
T = 1; % [s] prediction horizon length
x0 = [0; pi; 0; 0]; % initial state

%% model dynamics
model = cart_pole_model();
nx = length(model.x); % state size
nu = length(model.u); % input size

%% OCP formulation object
ocp = AcadosOcp();
ocp.model = model;

%% cost in nonlinear least squares form
W_x = diag([1e3, 1e3, 1e-2, 1e-2]);
W_u = 1e-2;

% initial cost term
ny_0 = nu;
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = W_u;
ocp.cost.yref_0 = zeros(ny_0, 1);
ocp.model.cost_y_expr_0 = model.u;

% path cost term
ny = nx + nu;
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = zeros(ny, 1);
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% terminal cost term
ny_e = nx;
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.model.cost_y_expr_e = model.x;
ocp.cost.yref_e = zeros(ny_e, 1);
ocp.cost.W_e = W_x;

%% define constraints
% only bound on u on initial stage and path
ocp.model.con_h_expr = model.u;
ocp.model.con_h_expr_0 = model.u;

U_max = 80;
ocp.constraints.lh = -U_max;
ocp.constraints.lh_0 = -U_max;
ocp.constraints.uh = U_max;
ocp.constraints.uh_0 = U_max;
ocp.constraints.x0 = x0;

% define solver options
% ocp.solver_options.N_horizon = N;
% ocp.solver_options.tf = T;
% ocp.solver_options.nlp_solver_type = 'SQP';
% ocp.solver_options.integrator_type = 'IRK';
% ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
% ocp.solver_options.qp_solver_mu0 = 1e3;
% ocp.solver_options.qp_solver_cond_N = 5;
% ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
% ocp.solver_options.ext_fun_compile_flags = '-O2';
% ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
% ocp.solver_options.qp_solver_iter_max = 100
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

% create solver
ocp_solver = AcadosOcpSolver(ocp);

try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('Solver creado exitosamente\n');
catch ME
    fprintf('Error creando solver: %s\n', ME.message);
    rethrow(ME);
end