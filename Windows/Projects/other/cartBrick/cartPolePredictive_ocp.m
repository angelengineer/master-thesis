%% cartPolePredictive_ocp.m
% MPC de péndulo sobre carro con parámetros online (p) para modificar pesos
% en tiempo real desde Simulink mediante la S-Function generada por acados.

import casadi.*

if ~exist('simulink_opts','var')
    disp('Using empty simulink_opts (Simulink block will be generated).');
    simulink_opts = [];
end

check_acados_requirements();

%% ------------------------------------------------------
%% Solver settings
%% ------------------------------------------------------
N = 20;          % control horizon
T = 1.0;         % prediction horizon [s]
x0 = [0; pi; 0; 0];

%% ------------------------------------------------------
%% Load model
%% ------------------------------------------------------
model = get_pendulum_on_cart_model();
nx = length(model.x);
nu = length(model.u);

%% ------------------------------------------------------
%% ONLINE PARAMETER p  (para cambiar pesos en Simulink)
%% ------------------------------------------------------
% p será un vector de 4 elementos, uno para cada estado.
p = SX.sym('p', nx);
model.p = p;

% Estados escalados por parámetros online
x_scaled = sqrt(p) .* model.x;

%% ------------------------------------------------------
%% Create OCP object
%% ------------------------------------------------------
ocp = AcadosOcp();
ocp.model = model;

% IMPORTANTE: la dimensión de parámetros se define AQUÍ
ocp.dims.np = nx;

%% ------------------------------------------------------
%% Define COST with online parameter p
%% ------------------------------------------------------

% Cost expressions
ocp.model.cost_y_expr    = [x_scaled; model.u];    % correr durante el horizonte
ocp.model.cost_y_expr_0  = model.u;                % coste inicial: solo u
ocp.model.cost_y_expr_e  = x_scaled;               % terminal

% Cost types
ocp.cost.cost_type     = 'NONLINEAR_LS';
ocp.cost.cost_type_0   = 'NONLINEAR_LS';
ocp.cost.cost_type_e   = 'NONLINEAR_LS';

% Dimensiones de las referencias
ny   = nx + nu;
ny_0 = nu;
ny_e = nx;

ocp.cost.yref     = zeros(ny, 1);
ocp.cost.yref_0   = zeros(ny_0, 1);
ocp.cost.yref_e   = zeros(ny_e, 1);

% Pesos: identidad (la escala real la define p)
ocp.cost.W     = eye(ny);
ocp.cost.W_0   = 1e-2 * eye(ny_0);
ocp.cost.W_e   = eye(ny_e);

%% ------------------------------------------------------
%% Constraints
%% ------------------------------------------------------
U_max = 80;

% Control bound
ocp.constraints.idxbu = 0;
ocp.constraints.lbu   = -U_max;
ocp.constraints.ubu   =  U_max;

% State bound
ocp.constraints.idxbx = 0;    % aplicar solo al estado "p"
ocp.constraints.lbx   = -5;
ocp.constraints.ubx   =  5;

% Terminal bound
ocp.constraints.idxbx_e = 0;
ocp.constraints.lbx_e   = -5;
ocp.constraints.ubx_e   =  5;

% Initial bound
ocp.constraints.idxbx_0 = 0;
ocp.constraints.lbx_0   = -5;
ocp.constraints.ubx_0   =  5;

% Initial condition
ocp.constraints.x0 = x0;

%% ------------------------------------------------------
%% Solver options
%% ------------------------------------------------------
ocp.solver_options.N_horizon       = N;
ocp.solver_options.tf              = T;
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_mu0   = 1e3;
ocp.solver_options.globalization   = 'MERIT_BACKTRACKING';
ocp.solver_options.hessian_approx  = 'GAUSS_NEWTON';

% Enable Simulink block
ocp.simulink_opts = simulink_opts;

%% ------------------------------------------------------
%% Generate OCP solver + S-Function code
%% ------------------------------------------------------
ocp_solver = AcadosOcpSolver(ocp);
