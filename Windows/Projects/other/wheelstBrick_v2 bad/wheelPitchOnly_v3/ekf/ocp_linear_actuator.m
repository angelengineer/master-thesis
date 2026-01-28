import casadi.*

%% ============================================
%% SOLVER SETTINGS
%% ============================================
if ~exist('simulink_opts','var')
    simulink_opts = [];
end

check_acados_requirements()

% Prediction horizon
N = 20;          % Number of control intervals
T = 2.0;         % Prediction horizon [s]
dt = T/N;        % Sampling time

% Initial state [position; velocity]
x0 = [0; 0];     % [m; m/s]

%% ============================================
%% LOAD MODEL
%% ============================================
model = get_linear_actuator_model();
nx = length(model.x);    % 2 states: [pos, vel]
nu = length(model.u);    % 1 control: F_applied
np = length(model.p);    % 2 parameters: [mass, pitch]

%% ============================================
%% OCP FORMULATION
%% ============================================
ocp = AcadosOcp();
ocp.model = model;

%% COST FUNCTION (NONLINEAR_LS)
% State weights: [pos, vel]
W_x = diag([1e3, 1e1]);  % High pos weight, moderate vel weight

% Control weight
W_u = 1e-3;              % Low control effort penalty

% Reference trajectory (constant for simplicity)
pos_ref = 1.0;           % Target position [m]
vel_ref = 0.0;           % Target velocity [m/s]
yref = [pos_ref; vel_ref; 0];  % [pos_ref, vel_ref, F_ref]

% Initial stage cost
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
ocp.cost.yref_0 = yref;
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% Path cost (stages 1 to N-1)
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = yref;
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% Terminal cost (stage N)
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.W_e = 10 * W_x;  % Higher terminal weight for stability
ocp.cost.yref_e = [pos_ref; vel_ref];
ocp.model.cost_y_expr_e = model.x;

%% CONSTRAINTS
% Control constraints (force limits)
F_min = 0.0;      % Minimum thrust [N] (e.g., no reverse thrust)
F_max = 100.0;    % Maximum thrust [N]

% State constraints
pos_min = -5.0;   % Position lower bound [m]
pos_max = 5.0;    % Position upper bound [m]
vel_min = -3.0;   % Velocity lower bound [m/s]
vel_max = 3.0;    % Velocity upper bound [m/s]

% Constraint type
ocp.constraints.constr_type = 'BGH';  % Box constraints

% Control constraints (all stages)
ocp.constraints.idxbu = 0;            % Index of F_applied
ocp.constraints.lbu = F_min;
ocp.constraints.ubu = F_max;

% State constraints (path stages 1 to N-1)
ocp.constraints.idxbx = 0:1;          % [pos, vel] indices
ocp.constraints.lbx = [pos_min; vel_min];
ocp.constraints.ubx = [pos_max; vel_max];

% Terminal state constraints (stage N)
ocp.constraints.idxbx_e = 0:1;
ocp.constraints.lbx_e = [pos_min; vel_min];
ocp.constraints.ubx_e = [pos_max; vel_max];

% Initial state constraint (stage 0)
ocp.constraints.idxbx_0 = 0:1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% SOLVER OPTIONS
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;

% Numerical settings
ocp.solver_options.nlp_solver_type = 'SQP';          % Sequential Quadratic Programming
ocp.solver_options.nlp_solver_max_iter = 100;        % Max SQP iterations
ocp.solver_options.nlp_solver_tol_stat = 1e-6;       % Stationarity tolerance
ocp.solver_options.nlp_solver_tol_eq = 1e-6;         % Equality constraint tolerance

% Integrator settings
ocp.solver_options.integrator_type = 'IRK';          % Implicit Runge-Kutta (more stable)
ocp.solver_options.sim_method_num_stages = 2;        % Gauss-Legendre stages
ocp.solver_options.sim_method_num_steps = 2;         % Steps per interval

% QP solver settings
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.qp_solver_iter_max = 50;


% Performance tuning
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';  % Efficient Hessian approximation
ocp.solver_options.print_level = 0;                  % 0=silent, 1=summary, 2=debug
ocp.solver_options.ext_fun_compile_flags = '-O3 -march=native';  % Compiler optimizations

%% PARAMETER HANDLING SETUP
% Initialize parameter values (will be updated at runtime)
% Format: [mass; pitch] for each stage
p_default = [10.0; 0.0];  % Default mass=10kg, pitch=0rad
ocp.parameter_values = p_default;

%% SIMULINK INTEGRATION
ocp.simulink_opts = simulink_opts;

%% ============================================
%% CREATE SOLVER
%% ============================================
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✅ MPC solver created successfully\n');
    fprintf('   Horizon: %d steps (%.2fs), dt=%.3fs\n', N, T, dt);
    fprintf('   States: [pos, vel], Control: F_applied\n');
    fprintf('   Parameters: [mass, pitch] (time-varying)\n');
catch ME
    fprintf('❌ Error creating solver: %s\n', ME.message);
    error('Solver creation failed');
end

