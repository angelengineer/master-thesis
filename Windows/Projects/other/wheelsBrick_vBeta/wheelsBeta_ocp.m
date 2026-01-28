import casadi.*

% ============================================================
%  ACADOS OCP – WHEELED BRICK (BETA-ONLY MPC)
% ============================================================



check_acados_requirements();

%% ================= SOLVER SETTINGS =================
N = 40;      % number of discretization steps
T = 1.0;     % prediction horizon [s]

% Estado inicial: [beta; beta_dot]
x0 = [deg2rad(5); 0];   % 5 grados de perturbación inicial

%% ================= MODEL =================
model = get_wheeled_beta_model();

nx = length(model.x);   % = 2
nu = length(model.u);   % = 1

fprintf('Modelo beta-only cargado: nx=%d, nu=%d\n', nx, nu);

%% ================= OCP OBJECT =================
ocp = AcadosOcp();
ocp.model = model;

%% ================= COST FUNCTION =================
% Pesos de estados: [beta, beta_dot]
W_x = diag([
    1e2;   % beta (MUY crítico)
    1e2    % beta_dot
]);

% Peso de control
W_u = 1e-2;

%% ----- COSTE INICIAL -----
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
ocp.cost.yref_0 = zeros(nx + nu, 1);
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

%% ----- COSTE DE TRAYECTORIA -----
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = zeros(nx + nu, 1);
ocp.model.cost_y_expr = vertcat(model.x, model.u);

%% ----- COSTE TERMINAL -----
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.W_e = 10 * W_x;
ocp.cost.yref_e = zeros(nx,1);
ocp.model.cost_y_expr_e = model.x;

%% ================= CONSTRAINTS =================
ocp.constraints.constr_type = 'BGH';

% ---- Control ----
U_max = 10;  % Nm (par total)
ocp.constraints.idxbu = 0;
ocp.constraints.lbu = -U_max;
ocp.constraints.ubu =  U_max;

% ---- Estados ----
beta_max = deg2rad(45);
beta_dot_max = deg2rad(300);

ocp.constraints.idxbx = 0:nx-1;
ocp.constraints.lbx = [-beta_max; -beta_dot_max];
ocp.constraints.ubx = [ beta_max;  beta_dot_max];

% ---- Estado inicial ----
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

% ---- Terminal ----
ocp.constraints.idxbx_e = 0:nx-1;
ocp.constraints.lbx_e = [-beta_max; -beta_dot_max];
ocp.constraints.ubx_e = [ beta_max;  beta_dot_max];

%% ================= SOLVER OPTIONS =================
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.qp_solver_warm_start = 2;
ocp.solver_options.nlp_solver_max_iter = 100;
ocp.solver_options.qp_solver_iter_max = 50;
ocp.solver_options.print_level = 1;
ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';


ocp.simulink_opts = simulink_opts;

%% ================= CREATE SOLVER =================
fprintf('Creando solver acados...\n');
ocp_solver = AcadosOcpSolver(ocp);
fprintf('Solver creado correctamente.\n');

%% ================= INITIAL GUESS =================
x_init = repmat(x0, 1, N+1);
u_init = zeros(nu, N);

ocp_solver.set('init_x', x_init);
ocp_solver.set('init_u', u_init);

%% ================= SOLVE =================
fprintf('Resolviendo OCP...\n');
ocp_solver.solve();

status = ocp_solver.get('status');
if status == 0
    fprintf('Solver convergió correctamente.\n');
else
    error('Solver falló con status %d', status);
end

%% ================= GET SOLUTION =================
xtraj = ocp_solver.get('x');
utraj = ocp_solver.get('u');

beta_traj     = xtraj(1,:);
beta_dot_traj = xtraj(2,:);

% Asignación de par a ruedas (post-procesado)
tau_L = utraj / 2;
tau_R = utraj / 2;

