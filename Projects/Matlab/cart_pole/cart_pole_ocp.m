import casadi.*

% === Configuración inicial ===
if ~exist('simulink_opts','var')
    simulink_opts = [];
end
check_acados_requirements()

%% solver settings
N = 40;                     % pasos de discretización
T = 1;                      % [s] horizonte de predicción
x0 = [0; pi; 0; 0];         % estado inicial: [x, theta, x_dot, theta_dot]
                            % theta = pi → péndulo colgando (equilibrio estable)

%% model dynamics
model = cart_pole_model();
nx = length(model.x);       % = 4
nu = length(model.u);       % = 1

%% OCP formulation
ocp = AcadosOcp();
ocp.model = model;

%% costes (NONLINEAR_LS)
W_x = diag([1e3, 1e3, 1e-2, 1e-2]);  % pesos: [x, theta, x_dot, theta_dot]
W_u = 1e-2;                          % peso control

% Costo inicial (stage 0)
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = W_u;
ocp.cost.yref_0 = 0;
ocp.model.cost_y_expr_0 = model.u;

% Costo de trayectoria (stages 1..N-1)
ny = nx + nu;
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = zeros(ny, 1);        % referencia: [0; pi; 0; 0; 0] → mantener colgando
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% Costo terminal (stage N) - OPCIONAL pero recomendado para estabilidad
% ny_e = nx;
% ocp.cost.cost_type_e = 'NONLINEAR_LS';
% ocp.cost.W_e = W_x;                  % mismo peso que estados
% ocp.cost.yref_e = [0; pi; 0; 0];     % referencia terminal = equilibrio estable
% ocp.model.cost_y_expr_e = model.x;

%% === RESTRICCIONES RECTANGULARES (BOX CONSTRAINTS) ===
% Tipo de restricciones: BGH = Box, General, H (nonlinear) → usamos Box
ocp.constraints.constr_type = 'BGH';

% --- Límites de CONTROL (u) ---
U_max = 80;                          % [N] fuerza máxima del carro
ocp.constraints.idxbu = 0;           % índice del control (0-based: u es primer/único input)
ocp.constraints.lbu = -U_max;
ocp.constraints.ubu =  U_max;

% --- Límites de ESTADO (x) ---
% Estados: [x, theta, x_dot, theta_dot]
x_max      = 2.5;                    % [m]   límite carril del carro
theta_max  = deg2rad(60);            % [rad] ±60° alrededor de pi (evita giros completos)
x_dot_max  = 3.0;                    % [m/s] velocidad máxima carro
theta_dot_max = deg2rad(180);        % [rad/s] velocidad angular máxima

% Índices de estados (0-based en acados)
ocp.constraints.idxbx = 0:nx-1;      % [0, 1, 2, 3] → todos los estados

% Límites para stages 1..N-1 (trayectoria)
ocp.constraints.lbx = [...
    -x_max;                          % x_min
    pi - theta_max;                  % theta_min (alrededor de pi)
    -x_dot_max;                      % x_dot_min
    -theta_dot_max];                 % theta_dot_min

ocp.constraints.ubx = [...
    x_max;                           % x_max
    pi + theta_max;                  % theta_max
    x_dot_max;                       % x_dot_max
    theta_dot_max];                  % theta_dot_max

% --- Estado inicial FIJO (stage 0) ---
ocp.constraints.idxbx_0 = 0:nx-1;    % fijar todos los estados iniciales
ocp.constraints.lbx_0 = x0;          % = ubx_0 → igualdad x(0) = x0
ocp.constraints.ubx_0 = x0;

% --- Límites terminales (stage N) - OPCIONAL pero recomendado ---
% Mismos límites que trayectoria para evitar comportamiento salvaje al final
ocp.constraints.idxbx_e = 0:nx-1;
ocp.constraints.lbx_e = ocp.constraints.lbx;
ocp.constraints.ubx_e = ocp.constraints.ubx;

%% solver options
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';      % rápido para RTI
ocp.solver_options.integrator_type = 'GNSF';          % eficiente para sistemas mecánicos
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.qp_solver_warm_start = 2;
ocp.solver_options.nlp_solver_max_iter = 10;
ocp.solver_options.qp_solver_iter_max = 20;
ocp.solver_options.print_level = 1;
ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';
ocp.solver_options.compile_interface = 'STATIC';
ocp.simulink_opts = simulink_opts;

%% crear solver
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✓ Solver creado exitosamente con restricciones rectangulares\n');
catch ME
    fprintf('✗ Error creando solver: %s\n', ME.message);
    rethrow(ME);
end