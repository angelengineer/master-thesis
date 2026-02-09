

clear all; clc;
%% get available simulink_opts with default options
simulink_opts = get_acados_simulink_opts;

check_acados_requirements()

% initial state
x0 = [0; 0; 0; 0];  % start at stable position

%% OCP DESCRIPTION
ocp = AcadosOcp();

%% IVP DESCRIPTION
sim = AcadosSim();

%% SOLVER OPTIONS

%% discretization
h = 0.01; % sampling time = length of first shooting interval
N = 20; % number of shooting intervals
% nonuniform discretization
shooting_nodes = [0.0 0.01, 0.05*(1:N-1)];
T = shooting_nodes(end);

ocp.solver_options.tf = T;
ocp.solver_options.N_horizon = N;
ocp.solver_options.shooting_nodes = shooting_nodes;
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
% FULL_CONDENSING_HPIPM, PARTIAL_CONDENSING_HPIPM
% FULL_CONDENSING_QPOASES, PARTIAL_CONDENSING_OSQP
ocp.solver_options.qp_solver_cond_N = 5; % for partial condensing
ocp.solver_options.globalization = 'MERIT_BACKTRACKING'; % turns on globalization
ocp.solver_options.nlp_solver_max_iter = 200;

% we add some model-plant mismatch by choosing different integration
% methods for model (within the OCP) and plant:

% integrator model
model_integrator_type = 'IRK';
model_sim_method_num_stages = 1;
model_sim_method_num_steps = 2;

ocp.solver_options.sim_method_num_stages = model_sim_method_num_stages;
ocp.solver_options.sim_method_num_steps = model_sim_method_num_steps;
ocp.solver_options.integrator_type = model_integrator_type;

% integrator plant
plant_integrator_type = 'IRK';
plant_sim_method_num_stages = 3;
plant_sim_method_num_steps = 3;

sim.solver_options.num_stages = plant_sim_method_num_stages;
sim.solver_options.num_steps = plant_sim_method_num_steps;
sim.solver_options.Tsim = h;
sim.solver_options.integrator_type = plant_integrator_type;

%% MODEL with mass as parameter
model = get_pendulum_on_cart_model(h, true);
ocp.model = model;
sim.model = model;

% dimensions
nx = model.x.rows();
nu = model.u.rows();

%% COST: nonlinear-least squares cost
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';

W_x = diag([1e3, 1e3, 1e-2, 1e-2]);
W_u = 1e-2;

model.cost_y_expr_0 = model.u;
model.cost_y_expr = vertcat(model.x, model.u);
model.cost_y_expr_e = model.x;

ocp.cost.W_0 = W_u;
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.W_e = W_x;

% initialize reference to zero, can be changed after solver creation
ocp.cost.yref_0 = zeros(size(model.cost_y_expr_0));
ocp.cost.yref = zeros(size(model.cost_y_expr));
ocp.cost.yref_e = zeros(size(model.cost_y_expr_e));

%% CONSTRAINTS

U_max = 80;
ocp.constraints.constr_type = 'AUTO';
ocp.constraints.constr_type_0 = 'AUTO';

model.con_h_expr_0 = model.u;
ocp.constraints.lh_0 = -U_max;
ocp.constraints.uh_0 = U_max;

model.con_h_expr = model.u;
ocp.constraints.lh = -U_max;
ocp.constraints.uh = U_max;

ocp.constraints.x0 = x0;


% simulink 
ocp.simulink_opts = simulink_opts;

%% OCP SOLVER
ocp_solver = AcadosOcpSolver(ocp);

% set parameter for all stages
for i = 0:N
    ocp_solver.set('p', 1., i);
end

%% SIM SOLVER/INTEGRATOR
sim_solver = AcadosSimSolver(sim);

% set parameter
sim_solver.set('p', 1.05); % model-plant mismatch in the parameters

%% Simulink blocks
cd c_generated_code

make_sfun_sim; % integrator
make_sfun; % ocp solver
cd ..