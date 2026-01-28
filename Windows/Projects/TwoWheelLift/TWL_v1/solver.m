import casadi.*
close all;

%% =========================
%% CONFIGURACIÓN
%% =========================
N  = 40;        % horizon steps
T  = 1.0;       % horizon [s]
dt = T/N;

x0 = [0; deg2rad(10); 0; 0];   % [x, beta, x_dot, beta_dot]
d_f_initial = 0.3;

%% =========================
%% CARGAR MODELO
%% =========================
[model, helpers] = get_model_1D_final();

nx = length(model.x);
nu = length(model.u);
np = length(model.p);

%% =========================
%% DEFINIR OCP
%% =========================
ocp = AcadosOcp();
ocp.model = model;

%% =========================
%% COSTE (NONLINEAR LS)
%% =========================
W_x = diag([1e-9, 1e4, 1e2, 1e3]);
W_u = 1e-3 * eye(nu);

% Initial
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);
ocp.cost.yref_0 = zeros(nx + nu, 1);

% Path
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.model.cost_y_expr = vertcat(model.x, model.u);
ocp.cost.yref = zeros(nx + nu, 1);

% % Terminal
% ocp.cost.cost_type_e = 'NONLINEAR_LS';
% ocp.cost.W_e = 10 * W_x;
% ocp.model.cost_y_expr_e = model.x;
% ocp.cost.yref_e = zeros(nx,1);

%% =========================
%% RESTRICCIONES
%% =========================
tau_max = 15;
beta_max = deg2rad(30);
x_dot_max = 2.0;
beta_dot_max = deg2rad(180);

ocp.constraints.constr_type = 'BGH';

% Control bounds
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -tau_max * ones(nu,1);
ocp.constraints.ubu =  tau_max * ones(nu,1);

% State bounds
ocp.constraints.idxbx = 0:nx-1;
ocp.constraints.lbx = [-inf; -beta_max; -x_dot_max; -beta_dot_max];
ocp.constraints.ubx = [ inf;  beta_max;  x_dot_max;  beta_dot_max];

% % Terminal bounds
% ocp.constraints.idxbx_e = 0:nx-1;
% ocp.constraints.lbx_e = [-inf; -deg2rad(1); -0.1; -0.1];
% ocp.constraints.ubx_e = [ inf;  deg2rad(1);  0.1;  0.1];

% Initial state
ocp.constraints.x0 = x0;

% Initial parameter value
ocp.parameter_values = d_f_initial;

%% =========================
%% OPCIONES SOLVER
%% =========================
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.nlp_solver_max_iter = 100;
ocp.solver_options.qp_solver_iter_max = 50;
ocp.solver_options.print_level = 0;
ocp.simulink_opts = simulink_opts;


%% =========================
%% CREAR SOLVER
%% =========================
ocp_solver = AcadosOcpSolver(ocp);