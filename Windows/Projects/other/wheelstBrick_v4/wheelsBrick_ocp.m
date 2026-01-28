import casadi.*

% options needed for the Simulink example
if ~exist('simulink_opts','var')
    disp('using empty simulink_opts to generate solver without simulink block')
    simulink_opts = [];
end

check_acados_requirements()

%% solver settings
N = 40; % number of discretization steps
T = 2; % [s] prediction horizon length
% Estado inicial 2D: [x, y, theta, beta, v, w, beta_dot]
x0 = [0; 0; 0; deg2rad(15); 0; 0; 0]; % Inclinado 15° para prueba realista

%% model dynamics (USA EL NUEVO MODELO 2D)
[model, model_params] = get_wheeled_brick_model_2D();
nx = length(model.x); % state size = 7
nu = length(model.u); % input size = 2

%% OCP formulation object
ocp = AcadosOcp();
ocp.model = model;

%% cost formulation - ¡CORREGIDO DEFINITIVO!
% ========== COSTO INICIAL (stage 0) - SOLUCIÓN ROBUSTA ==========
% Usamos LINEAR_LS con matrices CERO pero DIMENSIONES CONSISTENTES
ny_0 = 5; % Número de salidas para costo inicial (igual que path cost)
ocp.cost.cost_type_0 = 'LINEAR_LS';

% Matrices de peso CERO (no penaliza nada en estado inicial)
ocp.cost.W_0 = zeros(ny_0, ny_0); 

% Matrices de mapeo para extraer [beta, v, w, tau_L, tau_R]
ocp.cost.Vx_0 = zeros(ny_0, nx);
ocp.cost.Vu_0 = zeros(ny_0, nu);
ocp.cost.Vx_0(1,4) = 1; % beta (índice 4)
ocp.cost.Vx_0(2,5) = 1; % v (índice 5)
ocp.cost.Vx_0(3,6) = 1; % w (índice 6)
ocp.cost.Vu_0(4,1) = 1; % tau_L (índice 1)
ocp.cost.Vu_0(5,2) = 1; % tau_R (índice 2)

% Referencia inicial (no importa porque W_0=0)
v_ref = 0.3; w_ref = 0.0; beta_ref = 0.0;
ocp.cost.yref_0 = [beta_ref; v_ref; w_ref; 0; 0];

% ========== COSTO PATH (stages 1 to N-1) ==========
ocp.cost.cost_type = 'NONLINEAR_LS';
% Pesos SOLO para variables críticas: [beta, v, w, tau_L, tau_R]
W_path = diag([1e4,   % beta (pitch) - peso ALTO para estabilidad
               1e4,   % v (velocidad lineal)
               1e0,   % w (velocidad angular)
               1e-4,  % tau_L
               1e-4]);% tau_R

ocp.cost.yref = [beta_ref; v_ref; w_ref; 0; 0]; % [beta, v, w, tau_L_ref, tau_R_ref]

% Expresión del costo: SOLO variables críticas
ocp.model.cost_y_expr = vertcat(...
    model.x(4),...     % beta (pitch)
    model.x(5),...     % v (velocidad lineal)
    model.x(6),...     % w (velocidad angular)
    model.u);          % tau_L, tau_R

ocp.cost.W = W_path;

% ========== COSTO TERMINAL (stage N) ==========
ocp.cost.cost_type_e = 'NONLINEAR_LS';
W_terminal = diag([1e5, 1e2, 1e1]); % beta, v, w
ocp.cost.W_e = W_terminal;
ocp.cost.yref_e = [beta_ref; v_ref; w_ref];
ocp.model.cost_y_expr_e = vertcat(model.x(4), model.x(5), model.x(6)); % beta, v, w

%% constraints (CORREGIDAS Y SEGURAS)
% Límites de control
U_max = 20;  % Nm

% Usar restricciones de caja
ocp.constraints.constr_type = 'BGH';

% Restricciones de CONTROL
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -U_max * ones(nu,1);
ocp.constraints.ubu = U_max * ones(nu,1);

% Límites de estado CRÍTICOS
beta_max = deg2rad(30);  % Límite de seguridad absoluto
beta_dot_max = deg2rad(120);
v_max = 1.0;
w_max = deg2rad(60);

% Restricciones para etapas de camino (1 a N-1)
ocp.constraints.idxbx = [4, 5, 6, 7]; % beta, v, w, beta_dot
ocp.constraints.lbx = [-beta_max; -v_max; -w_max; -beta_dot_max];
ocp.constraints.ubx = [beta_max; v_max; w_max; beta_dot_max];

% Restricciones TERMINALES más estrictas
ocp.constraints.idxbx_e = [4, 5, 6, 7];
ocp.constraints.lbx_e = [-deg2rad(5); -0.1; -deg2rad(10); -beta_dot_max/2];
ocp.constraints.ubx_e = [deg2rad(5); 0.1; deg2rad(10); beta_dot_max/2];

% Estado inicial FIJO (TODOS los estados)
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% solver options (ESTABLES)
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.integrator_type = 'IRK';
ocp.solver_options.sim_method_num_stages = 2;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.nlp_solver_max_iter = 50;
ocp.solver_options.qp_solver_iter_max = 100;
ocp.solver_options.levenberg_marquardt = 1e-3;
ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';

ocp.simulink_opts = simulink_opts;

%% create solver - ¡AHORA FUNCIONARÁ!
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✓ Solver 2D creado exitosamente\n');
catch ME
    fprintf('✗ Error creando solver 2D: %s\n', ME.message);
    % Mostrar dimensiones críticas para debug
    fprintf('\nDIMENSIONES DEL COSTO INICIAL:\n');
    fprintf('  ny_0 = %d\n', ny_0);
    fprintf('  size(W_0) = [%d, %d]\n', size(ocp.cost.W_0,1), size(ocp.cost.W_0,2));
    fprintf('  size(Vx_0) = [%d, %d]\n', size(ocp.cost.Vx_0,1), size(ocp.cost.Vx_0,2));
    fprintf('  size(Vu_0) = [%d, %d]\n', size(ocp.cost.Vu_0,1), size(ocp.cost.Vu_0,2));
    fprintf('  size(yref_0) = [%d, %d]\n', size(ocp.cost.yref_0,1), size(ocp.cost.yref_0,2));
    rethrow(ME);
end

%% solver initial guess INTELIGENTE
x_traj_init = repmat(x0, 1, N+1);
u_traj_init = zeros(nu, N);

% Trayectoria inicial para pitch
for i = 1:N+1
    alpha = (i-1)/N;
    x_traj_init(4,i) = x0(4) * (1 - alpha^2);
    x_traj_init(5,i) = v_ref * (1 - exp(-3*alpha));
    x_traj_init(3,i) = x0(3); % theta constante
end

% Torques iniciales basados en equilibrio estático
if abs(x0(4)) > deg2rad(5)
    torque_grav = model_params.M_body * model_params.g * model_params.d * sin(x0(4));
    u_traj_init(1,:) = -0.6 * torque_grav;
    u_traj_init(2,:) = -0.6 * torque_grav;
end

%% call ocp solver
fprintf('\nResolviendo OCP...\n');
t0 = tic;

% Inicialización correcta
for i=0:N
    ocp_solver.set(i, 'x', x_traj_init(:,i+1));
end
for i=0:N-1
    ocp_solver.set(i, 'u', u_traj_init(:,i+1));
end

% solve
status = ocp_solver.solve();
solve_time = toc(t0)*1000; % ms

fprintf('Solver status: %d | Tiempo: %.1f ms\n', status, solve_time);

% get solution
xtraj = zeros(nx, N+1);
utraj = zeros(nu, N);

for i=0:N
    xtraj(:,i+1) = ocp_solver.get(i, 'x');
end
for i=0:N-1
    utraj(:,i+1) = ocp_solver.get(i, 'u');
end

% Resto del código (gráficos, análisis) igual que antes...