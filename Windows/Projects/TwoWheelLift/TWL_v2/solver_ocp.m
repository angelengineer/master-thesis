import casadi.*

%% ============================================
%% 1. CARGAR MODELO DINÁMICO
%% ============================================
model = get_wheeled_robot_model();
nx = size(model.x, 1);    % 8 estados
nu = size(model.u, 1);    % ahora 3 controles (tau_x, tau_m, tau_a)

%% ============================================
%% 2. PARÁMETROS DEL MPC
%% ============================================
Ts = 0.05;                 % Tiempo de muestreo [s]
N = 50;                   % Horizonte de predicción (2 segundos)
Tf = Ts * N;              % Tiempo total de predicción

x0 = zeros(nx,1);

%% ============================================
%% 3. MATRICES DE PESO (AJUSTAR SEGÚN NECESIDAD)
%% ============================================
% Pesos 
Q_pos = diag([10, 1000, 50, 30]);     % Pitch angle weight x5
Q_vel = diag([5, 2500, 20, 15]);      % Pitch velocity weight x5
% Matriz Q completa (estados)
Q = blkdiag(Q_pos, Q_vel);

% Pesos para CONTROLES (SOLO 3 ACTUADORES: tau_x, tau_m, tau_a)
R = diag([0.1,    % tau_x
          0.3,    % tau_m
          0.2]);  % tau_a

% Matriz terminal
Qn = 10 * Q;

%% ============================================
%% 4. RESTRICCIONES FÍSICAS (SEGURIDAD CRÍTICA)
%% ============================================
% Límites de POSICIÓN
x_min = [-inf;      % x
         -0.35;     % theta_p
         0.0;       % d_m
         -pi/3];    % theta_a

x_max = [inf;       % x
         0.35;      % theta_p
         1.2;       % d_m
         pi/3];     % theta_a

% Límites de VELOCIDAD
dx_min = [-1.5;     % dx
          -0.8;     % dtheta_p
          -0.3;     % dd_m
          -0.5];    % dtheta_a

dx_max = [1.5;      % dx
          0.8;      % dtheta_p
          0.3;      % dd_m
          0.5];     % dtheta_a

% Combinar en límites de estado completo
lbx = [x_min; dx_min];
ubx = [x_max; dx_max];

% Límites de CONTROLES (SOLO 3 actuadores: ¡se eliminó tau_p!)
lbu = [-50;   % tau_x
       -100;  % tau_m
       -20];  % tau_a

ubu = [50;    % tau_x
       100;   % tau_m
       20];   % tau_a

%% ============================================
%% 5. CONFIGURACIÓN DEL SOLVER ACADOS
%% ============================================
ocp = AcadosOcp();
ocp.model = model;

% Costo cuadrático
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';

% Matrices de peso
ocp.cost.W_0 = blkdiag(Q, R);
ocp.cost.W = blkdiag(Q, R);
ocp.cost.W_e = Qn;

% Referencias
ocp.cost.yref_0 = zeros(nx + nu, 1);
ocp.cost.yref = zeros(nx + nu, 1);
ocp.cost.yref_e = zeros(nx, 1);

ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);
ocp.model.cost_y_expr = vertcat(model.x, model.u);
ocp.model.cost_y_expr_e = model.x;

% Restricciones de caja
ocp.constraints.constr_type = 'BGH';

ocp.constraints.idxbx = 0:nx-1;  
ocp.constraints.lbx = lbx;
ocp.constraints.ubx = ubx;

ocp.constraints.idxbu = 0:nu-1;  % ahora 0,1,2 → 3 controles
ocp.constraints.lbu = lbu;
ocp.constraints.ubu = ubu;

% Estado inicial fijo
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% ============================================
%% 6. OPCIONES DE SOLVER (TIEMPO REAL)
%% ============================================
ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.tf = Tf;
ocp.solver_options.N_horizon = N;

ocp.solver_options.nlp_solver_max_iter = 500;
ocp.solver_options.qp_solver_iter_max = 500;

ocp.simulink_opts = simulink_opts;

%%old
% ocp.solver_options.N_horizon = N;
% ocp.solver_options.tf = Tf;
% ocp.solver_options.nlp_solver_type = 'SQP';
% ocp.solver_options.integrator_type = 'ERK';
% ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
% ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
% ocp.solver_options.qp_solver_cond_N = 5;
% ocp.solver_options.qp_solver_warm_start = 2;
% ocp.solver_options.nlp_solver_max_iter = 100;
% ocp.solver_options.qp_solver_iter_max = 50;
% ocp.solver_options.print_level = 1;
% ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';
% 
% ocp.simulink_opts = simulink_opts;

%% create solver
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('Solver creado exitosamente\n');
catch ME
    fprintf('Error creando solver: %s\n', ME.message);
    rethrow(ME);
end