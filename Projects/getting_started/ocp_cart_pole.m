
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
N = 20; % number of discretization steps
T = 1; % [s] prediction horizon length
x0 = [0; pi; 0; 0]; % initial state

%% model dynamics
model = model_cart_pole();
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


%% constraints
% Límites de control (fuerza realista)
F_max = 80;  % N

% Usar restricciones de caja
ocp.constraints.constr_type = 'BGH';

% Restricciones de CONTROL
ocp.constraints.idxbu = 0;  % índice 0 para el primer (y único) control
ocp.constraints.lbu = -F_max;
ocp.constraints.ubu = F_max;

% Límites de estado - para swing-up necesitamos permitir movimiento completo
p_max = 2.0;              % m - rango de posición
theta_max = 10*pi;        % rad - permitir múltiples rotaciones para swing-up
v_max = 10.0;             % m/s - velocidad máxima para swing-up
omega_max = 20*pi;        % rad/s - velocidad angular máxima para swing-up

% Para etapas de trayectoria (1 a N-1) - relajadas para swing-up
ocp.constraints.idxbx = 0:nx-1;  % [0, 1, 2, 3] para 4 estados
ocp.constraints.lbx = [-p_max; -theta_max; -v_max; -omega_max];
ocp.constraints.ubx = [p_max; theta_max; v_max; omega_max];

% Para etapa terminal (N) - relajadas también
ocp.constraints.idxbx_e = 0:nx-1;
ocp.constraints.lbx_e = [-p_max; -pi; -v_max; -omega_max];  % Permite cualquier ángulo terminal
ocp.constraints.ubx_e = [p_max; pi; v_max; omega_max];

% Para etapa inicial (0) - fijar el estado inicial
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% Configuración inicial para el solver
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
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_mu0 = 1e3;
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.ext_fun_compile_flags = '-O2';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
% ocp.solver_options.qp_solver_iter_max = 100
ocp.simulink_opts = simulink_opts;
ocp.code_export_directory = 'c_generated_code';

%% create solver
fprintf('\n=== Creando solver de Acados ===\n');

try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✓ Solver creado exitosamente\n');
    
catch ME
    fprintf('✗ Error creando o ejecutando solver: %s\n', ME.message);
    
    % Mostrar el stack trace para debugging
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  File: %s, Line: %d, Function: %s\n', ...
                ME.stack(i).file, ME.stack(i).line, ME.stack(i).name);
    end
    
    rethrow(ME);
end
