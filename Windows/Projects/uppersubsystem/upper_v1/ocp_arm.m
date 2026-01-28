import casadi.*

%% ============================================
%% SOLVER SETTINGS
%% ============================================
if ~exist('simulink_opts','var')
    simulink_opts = [];
end

check_acados_requirements()

% Horizon
N = 20;          % Number of control intervals
T = 2.0;         % Prediction horizon [s]
dt = T / N;

% Initial state [theta; theta_dot]
x0 = [0; 0];     % [rad; rad/s]

%% ============================================
%% LOAD MODEL
%% ============================================
model = get_arm_model();

nx = length(model.x);    % 2 states: [theta, theta_dot]
nu = length(model.u);    % 1 control: tau
np = length(model.p);    % 2 parameters: [mass, r]

%% ============================================
%% OCP FORMULATION
%% ============================================
ocp = AcadosOcp();
ocp.model = model;

%% ============================================
%% COST FUNCTION (NONLINEAR_LS)
%% ============================================
% State weights: [theta, theta_dot]
W_x = diag([5e3, 5e1]);

% Control weight
W_u = 1e-2;

% Reference
theta_ref     = pi/4;    % Desired angle [rad]
theta_dot_ref = 0.0;     % Desired angular velocity [rad/s]

yref = [theta_ref; theta_dot_ref; 0];  % [theta, theta_dot, tau]

% Initial stage
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
ocp.cost.yref_0 = yref;
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% Path stages
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = yref;
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% Terminal stage
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.W_e = 10 * W_x;
ocp.cost.yref_e = [theta_ref; theta_dot_ref];
ocp.model.cost_y_expr_e = model.x;

%% ============================================
%% CONSTRAINTS
%% ============================================
% Torque limits
tau_min = -20.0;   % [Nm]
tau_max =  20.0;   % [Nm]

% State limits
theta_min     = -pi;
theta_max     =  pi;
theta_dot_min = -5.0;
theta_dot_max =  5.0;

% Constraint type
ocp.constraints.constr_type = 'BGH';

% Control bounds
ocp.constraints.idxbu = 0;
ocp.constraints.lbu = tau_min;
ocp.constraints.ubu = tau_max;

% State bounds (path)
ocp.constraints.idxbx = 0:1;
ocp.constraints.lbx = [theta_min; theta_dot_min];
ocp.constraints.ubx = [theta_max; theta_dot_max];

% Terminal bounds
ocp.constraints.idxbx_e = 0:1;
ocp.constraints.lbx_e = [theta_min; theta_dot_min];
ocp.constraints.ubx_e = [theta_max; theta_dot_max];

% Initial state
ocp.constraints.idxbx_0 = 0:1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% ============================================
%% SOLVER OPTIONS
%% ============================================
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;

% NLP solver
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.nlp_solver_max_iter = 100;
ocp.solver_options.nlp_solver_tol_stat = 1e-6;
ocp.solver_options.nlp_solver_tol_eq = 1e-6;

% Integrator
ocp.solver_options.integrator_type = 'IRK';
ocp.solver_options.sim_method_num_stages = 2;
ocp.solver_options.sim_method_num_steps = 2;

% QP solver
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.qp_solver_iter_max = 50;

% Performance
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.print_level = 0;
ocp.solver_options.ext_fun_compile_flags = '-O3 -march=native';

%% ============================================
%% PARAMETER HANDLING
%% ============================================
% Parameters: [mass; r]
p_default = [5.0; 0.4];   % mass [kg], r [m]
ocp.parameter_values = p_default;

%% SIMULINK
ocp.simulink_opts = simulink_opts;

%% ============================================
%% CREATE SOLVER
%% ============================================
try
    ocp.code_export_directory = 'c_generated_code_arm_model';

    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✅ MPC solver created successfully\n');
    fprintf('   States: [theta, theta_dot]\n');
    fprintf('   Control: tau\n');
    fprintf('   Parameters: [mass, r]\n');
catch ME
    fprintf('❌ Error creating solver: %s\n', ME.message);
    error('Solver creation failed');
end
