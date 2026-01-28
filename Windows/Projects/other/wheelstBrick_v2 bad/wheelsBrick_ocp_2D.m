import casadi.*

% options needed for the Simulink example
if ~exist('simulink_opts','var')
    disp('using empty simulink_opts to generate solver without simulink block')
    simulink_opts = [];
end

check_acados_requirements()

%% solver settings
N = 40; % number of discretization steps
T = 2; % [s] prediction horizon length (más tiempo para 2D)
% Estado inicial: [x, y, α, v, v_lat, β, ω, β̇]
x0 = [0; 0; 0; 0; 0; 0; 0; 0]; 

%% model dynamics - USANDO EL MODELO 2D COMPLETO
model = get_wheeled_brick_model_2D();
nx = length(model.x); % state size = 8
nu = length(model.u); % input size = 2

fprintf('Modelo 2D cargado: %d estados, %d controles\n', nx, nu);

%% OCP formulation object
ocp = AcadosOcp();
ocp.model = model;

%% cost formulation - USING NONLINEAR_LS
% Pesos para estados: [x, y, α, v, v_lat, β, ω, β̇]
% x,y: posición (peso medio) 
% α: orientación (peso alto para mantener dirección)
% v: velocidad longitudinal (peso bajo, queremos movernos)
% v_lat: velocidad lateral (peso alto, queremos minimizar derrape)
% β: pitch angle (peso muy alto para estabilidad)
% ω: yaw rate (peso medio)
% β̇: pitch rate (peso medio)

W_x = diag([1, 1, 1, 1e0, 1e3, 1e4, 1e1, 1e2]);

% Pesos para controles: [τ_L, τ_R]
W_u = 1e-2 * eye(nu);

% ========== COSTO INICIAL (stage 0) ==========
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
% Referencia inicial: mantener estado actual (usualmente igual a x0)
ocp.cost.yref_0 = [x0; zeros(nu, 1)];
% Expresión del costo: [model.x; model.u]
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% ========== COSTO PATH (stages 1 to N-1) ==========
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);

% Definir una referencia de trayectoria simple
% Queremos: moverse en línea recta a 1 m/s, mantener brick vertical (β=0)
% sin deslizamiento lateral (v_lat=0), sin rotación (ω=0)
y_ref = [0;    % x - no nos importa la posición exacta
         0;    % y - mantener en línea recta
         0;    % α - mantener orientación inicial
         1.0;  % v - velocidad deseada = 1 m/s
         0;    % v_lat - minimizar derrape
         0;    % β - brick vertical
         0;    % ω - sin rotación
         0;    % β̇ - sin oscilación
         zeros(nu,1)]; % controles = 0

ocp.cost.yref = y_ref;
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% ========== COSTO TERMINAL (stage N) ==========
ocp.cost.cost_type_e = 'NONLINEAR_LS';
% Peso terminal mayor para asegurar estabilización
W_e = 5 * W_x;  % 5x más peso en el costo terminal
ocp.cost.W_e = W_e;
% Referencia terminal: mismos estados que en el path pero sin controles
y_ref_e = y_ref(1:nx);
ocp.cost.yref_e = y_ref_e;
% Expresión del costo terminal: solo estados
ocp.model.cost_y_expr_e = model.x;

%% constraints
% Límites de control (torques realistas)
U_max = 15;  % Nm (un poco más para 2D)

% Usar restricciones de caja
ocp.constraints.constr_type = 'BGH';

% Restricciones de CONTROL
ocp.constraints.idxbu = 0:nu-1;  % [0, 1] para 2 controles
ocp.constraints.lbu = -U_max * ones(nu,1);
ocp.constraints.ubu = U_max * ones(nu,1);

% Límites de estado para 8 estados
x_max = 10;          % posición x máxima [m]
y_max = 10;          % posición y máxima [m]
alpha_max = pi;      % orientación máxima [rad] (180°)
v_max = 3;           % velocidad longitudinal máxima [m/s]
v_lat_max = 0.5;     % velocidad lateral máxima [m/s] (minimizar derrape)
beta_max = deg2rad(30);  % ángulo de pitch máximo [rad] (30°)
omega_max = deg2rad(90); % velocidad angular de yaw máxima [rad/s]
beta_dot_max = deg2rad(180); % velocidad angular de pitch máxima [rad/s]

% Para etapas de trayectoria (1 a N-1)
ocp.constraints.idxbx = 0:nx-1;  % [0..7] para 8 estados
ocp.constraints.lbx = [-x_max; -y_max; -alpha_max; -v_max; -v_lat_max; 
                       -beta_max; -omega_max; -beta_dot_max];
ocp.constraints.ubx = [x_max; y_max; alpha_max; v_max; v_lat_max; 
                       beta_max; omega_max; beta_dot_max];

% Para etapa terminal (N)
ocp.constraints.idxbx_e = 0:nx-1;
ocp.constraints.lbx_e = [-x_max; -y_max; -alpha_max; -v_max; -v_lat_max; 
                         -beta_max; -omega_max; -beta_dot_max];
ocp.constraints.ubx_e = [x_max; y_max; alpha_max; v_max; v_lat_max; 
                         beta_max; omega_max; beta_dot_max];

% Para etapa inicial (0) - fijar el estado inicial
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% solver options
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';  % Más rápido para tiempo real
ocp.solver_options.integrator_type = 'ERK';      % Runge-Kutta explícito
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.qp_solver_cond_N = 5;
ocp.solver_options.qp_solver_warm_start = 2;
ocp.solver_options.nlp_solver_max_iter = 50;     % Menos iteraciones para más velocidad
ocp.solver_options.qp_solver_iter_max = 30;
ocp.solver_options.print_level = 1;
ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC';

% Opciones para estabilidad numérica
ocp.solver_options.nlp_solver_tol_stat = 1e-4;
ocp.solver_options.nlp_solver_tol_eq = 1e-4;
ocp.solver_options.nlp_solver_tol_ineq = 1e-4;
ocp.solver_options.nlp_solver_tol_comp = 1e-4;

ocp.simulink_opts = simulink_opts;

%% create solver
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('Solver 2D creado exitosamente\n');
    
    % Configurar opciones adicionales del solver
    ocp_solver.set('rti_phase', 0);  % 0=preparación, 1=feedback, 2=feedforward
    fprintf('Solver configurado para %d estados, %d controles\n', nx, nu);
    
catch ME
    fprintf('Error creando solver 2D: %s\n', ME.message);
    rethrow(ME);
end

%% solver initial guess
% Inicialización mejorada para 8 estados
x_traj_init = repmat(x0, 1, N+1);
u_traj_init = zeros(nu, N);

% Crear una trayectoria inicial simple: aceleración suave a velocidad deseada
for i = 1:N+1
    alpha_i = (i-1)/N;
    
    % Suavizar transiciones
    x_traj_init(4, i) = 1.0 * alpha_i;  % v: de 0 a 1 m/s
    x_traj_init(6, i) = 0;              % β: mantener en 0
    x_traj_init(7, i) = 0;              % ω: sin rotación
    x_traj_init(8, i) = 0;              % β̇: sin oscilación
    
    % Cinemática: estimar posición basada en velocidad
    if i > 1
        dt = T/N;
        x_traj_init(1, i) = x_traj_init(1, i-1) + x_traj_init(4, i-1) * cos(x_traj_init(3, i-1)) * dt;
        x_traj_init(2, i) = x_traj_init(2, i-1) + x_traj_init(4, i-1) * sin(x_traj_init(3, i-1)) * dt;
    end
end

% Inicialización de controles: pequeños torques para mantener velocidad
u_traj_init(1, :) = 0.1;  % τ_L pequeño positivo
u_traj_init(2, :) = 0.1;  % τ_R pequeño positivo

%% call ocp solver
% set trajectory initialization
ocp_solver.set('init_x', x_traj_init);
ocp_solver.set('init_u', u_traj_init);



% solve
fprintf('\n=== Resolviendo OCP 2D ===\n');
tic;
ocp_solver.solve();
solve_time = toc;
fprintf('Tiempo de solución: %.3f segundos\n', solve_time);

% get solution
utraj = zeros(nu, N);
xtraj = zeros(nx, N+1);

utraj = ocp_solver.get('u');
xtraj = ocp_solver.get('x');

status = ocp_solver.get('status');
if status == 0
    fprintf('✓ Solver succeeded!\n');
else
    fprintf('✗ Solver failed with status %d\n', status);
end

% Estadísticas
try
    ocp_solver.print('stat')
catch
    fprintf('No se pudieron imprimir estadísticas\n');
end

%% Análisis de la solución
fprintf('\n=== ANÁLISIS DE LA SOLUCIÓN 2D ===\n');
fprintf('Estado inicial:\n');
fprintf('  Posición: (%.3f, %.3f) m\n', x0(1), x0(2));
fprintf('  Orientación: %.2f°\n', rad2deg(x0(3)));
fprintf('  Velocidad: v=%.3f m/s, v_lat=%.3f m/s\n', x0(4), x0(5));
fprintf('  Pitch: β=%.2f°, β̇=%.2f°/s\n', rad2deg(x0(6)), rad2deg(x0(8)));
fprintf('  Yaw rate: ω=%.2f°/s\n', rad2deg(x0(7)));

fprintf('\nEstado final:\n');
fprintf('  Posición: (%.3f, %.3f) m\n', xtraj(1,end), xtraj(2,end));
fprintf('  Orientación: %.2f°\n', rad2deg(xtraj(3,end)));
fprintf('  Velocidad: v=%.3f m/s, v_lat=%.3f m/s\n', xtraj(4,end), xtraj(5,end));
fprintf('  Pitch: β=%.2f°, β̇=%.2f°/s\n', rad2deg(xtraj(6,end)), rad2deg(xtraj(8,end)));
fprintf('  Yaw rate: ω=%.2f°/s\n', rad2deg(xtraj(7,end)));

% Calcular error respecto a referencia
v_error = abs(xtraj(4,end) - y_ref(4));
v_lat_error = abs(xtraj(5,end) - y_ref(5));
beta_error = abs(rad2deg(xtraj(6,end)));

fprintf('\nErrores respecto a referencia:\n');
fprintf('  Error velocidad: %.4f m/s\n', v_error);
fprintf('  Error velocidad lateral: %.4f m/s\n', v_lat_error);
fprintf('  Error pitch: %.2f°\n', beta_error);

% Calcular energía de control
control_energy = sum(sum(utraj.^2)) * (T/N);
fprintf('Energía de control total: %.4f J\n', control_energy);

% Calcular distancia recorrida
dx = diff(xtraj(1,:));
dy = diff(xtraj(2,:));
distance = sum(sqrt(dx.^2 + dy.^2));
fprintf('Distancia recorrida estimada: %.3f m\n', distance);
