% =========================================================================
%  ACADOS OCP – WHEELED INVERTED PENDULUM (9-STATE - VERSIÓN FINAL)
% =========================================================================

clear; clc; close all;
check_acados_requirements();
import casadi.*

%% ================= MODELO =================
model = get_wheeled_brick_model_2D();
nx = length(model.x);   % = 9
nu = length(model.u);   % = 2
fprintf('Modelo cargado: nx=%d, nu=%d\n', nx, nu);

%% ================= HORIZON =================
N = 40;     % steps
T = 2.0;    % seconds

%% ================= ESTADO INICIAL =================
x0 = zeros(nx,1);

%% ================= OCP =================
ocp = AcadosOcp();
ocp.model = model;

%% ================= COST FUNCTION =================
W_x = diag([1.0, 1.0, 5.0, 1000.0, 1.0, 1.0, 10.0, 0.1, 0.1]);
W_u = diag([0.1; 0.1]);

% ----- Initial cost -----
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
ocp.cost.yref_0 = zeros(nx+nu, 1);
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% ----- Stage cost -----
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = zeros(nx+nu, 1);
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% ----- Terminal cost -----
ocp.cost.cost_type_e = 'NONLINEAR_LS';
W_e = 10 * W_x;
ocp.cost.W_e = W_e;
ocp.cost.yref_e = zeros(nx, 1);
ocp.model.cost_y_expr_e = model.x;

%% ================= CONSTRAINTS =================
ocp.constraints.constr_type = 'BGH';

% Input constraints
U_max = 20;
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -U_max * ones(nu,1);
ocp.constraints.ubu = U_max * ones(nu,1);

% State constraints
x_max = 10;
theta_max = pi;
phi_p_max = deg2rad(45);
phi_dot_max = 10;

ocp.constraints.idxbx = 0:nx-1;
ocp.constraints.lbx = [
    -x_max; -x_max; -theta_max; -phi_p_max;
    -inf; -inf; -phi_dot_max; -phi_dot_max; -phi_dot_max
];
ocp.constraints.ubx = -ocp.constraints.lbx;

% ¡IMPORTANTE! No definas restricciones iniciales si quieres poder cambiar x0 fácilmente
% En su lugar, inicializa la solución con x0_pert
% ocp.constraints.idxbx_0 = 0:nx-1;  ❌ COMENTADO
% ocp.constraints.lbx_0 = x0;        ❌ COMENTADO  
% ocp.constraints.ubx_0 = x0;        ❌ COMENTADO

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

%% ================= CREATE SOLVER =================
fprintf('\nCreando solver acados...\n');
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✓ Solver creado exitosamente\n');
catch ME
    fprintf('✗ Error creando solver: %s\n', ME.message);
    rethrow(ME);
end

%% ================= SIMULACIÓN =================
% Estado inicial perturbado
x0_pert = zeros(nx,1);
x0_pert(4) = deg2rad(5);  % 5 grados de pitch

fprintf('\nEstado inicial: Pitch = %.2f°\n', rad2deg(x0_pert(4)));

% Inicialización de trayectoria
x_init = repmat(x0_pert, 1, N+1);
u_init = zeros(nu, N);

% Suavizar inicialización
for i = 1:N+1
    alpha = (i-1)/N;
    x_init(4,i) = x0_pert(4) * (1 - alpha);  % Reducir pitch linealmente
end

ocp_solver.set('init_x', x_init);
ocp_solver.set('init_u', u_init);

%% ================= RESOLVER =================
fprintf('\nResolviendo OCP...\n');
ocp_solver.solve();

status = ocp_solver.get('status');
if status == 0
    fprintf('✓ Solver succeeded!\n');
else
    fprintf('✗ Solver failed with status %d\n', status);
end

%% ================= RESULTADOS =================
xtraj = ocp_solver.get('x');
utraj = ocp_solver.get('u');

fprintf('\n=== RESULTADOS ===\n');
fprintf('Pitch inicial: %.2f°\n', rad2deg(x0_pert(4)));
fprintf('Pitch final:   %.2f°\n', rad2deg(xtraj(4,end)));

%% ================= PLOTS =================
% ... (código de gráficas sin cambios)