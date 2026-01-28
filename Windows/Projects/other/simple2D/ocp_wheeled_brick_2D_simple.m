% OCP SETUP PARA WHEELED BRICK 2D (Pitch + Yaw)
% Basado en tu código 1D funcionando
import casadi.*

% Verificar requisitos
check_acados_requirements()

%% ============================================
%% CONFIGURACIÓN DEL SOLVER
%% ============================================
N = 40;   % pasos de discretización
T = 2.0;  % horizonte de predicción [s] (más largo para maniobras 2D)


%% ============================================
%% MODELO DINÁMICO
%% ============================================
[model, model_params, helpers] = get_wheeled_brick_2D_simple();
%% CONFIGURACIÓN INICIAL (2D)
nx = 7; % [x, y, alpha, beta, v, omega, beta_dot]
nu = 2; % [tau_L, tau_R]
x0 = [0; 0; 0; deg2rad(10); 0; 0; 0]; % Inclina brick 10° inicialmente

%% ============================================
%% OCP FORMULATION
%% ============================================
ocp = AcadosOcp();
ocp.model = model;

%% FUNCIÓN DE COSTO - NONLINEAR_LS
 % [x, y, alpha, beta, v, omega, beta_dot]
W_x = diag([1e-9, 1e-9, 1e-9, 1e3, 1e3, 1e3, 1e4]); % Pesos por estado
W_u = 1e-3 * eye(nu); % Pesos controles

% ========== COSTO INICIAL (stage 0) ==========
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.W_0 = blkdiag(W_x, W_u);
ocp.cost.yref_0 = zeros(nx+nu, 1);
ocp.model.cost_y_expr_0 = vertcat(model.x, model.u);

% ========== COSTO PATH (stages 1 to N-1) ==========
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.cost.W = blkdiag(W_x, W_u);

% REFERENCIA: Objetivo del control
% Ejemplo: ir a (x=2, y=1), orientado 45°, con β=0 (vertical)
% Referencia: CM estable (beta=0), v=0.5 m/s, omega=0.2 rad/s (giro suave)
yref = [0; 0; 0; 0; 0.5; 0.2; 0; 0; 0]; % [estados; controles]
ocp.cost.yref = yref;        % controles a cero

ocp.model.cost_y_expr = vertcat(model.x, model.u);

% ========== COSTO TERMINAL (opcional, comentado por ahora) ==========
% ocp.cost.cost_type_e = 'NONLINEAR_LS';
% W_e = 10 * W_x;
% ocp.cost.W_e = W_e;
% ocp.cost.yref_e = [x_ref; y_ref; alpha_ref; beta_ref; 0; 0; 0; 0];
% ocp.model.cost_y_expr_e = model.x;

%% ============================================
%% RESTRICCIONES
%% ============================================
U_max = 10;  % Torque máximo [Nm]

ocp.constraints.constr_type = 'BGH';

% Restricciones de control
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -U_max * ones(nu,1);
ocp.constraints.ubu = U_max * ones(nu,1);

% Límites de estado
x_max = 10;               % rango X [m]
y_max = 10;               % rango Y [m]
alpha_max = pi;           % ±180° yaw
beta_max = deg2rad(45);   % ±45° pitch
x_dot_max = 3;            % velocidad lineal máxima [m/s]
y_dot_max = 3;
alpha_dot_max = deg2rad(180);  % velocidad angular yaw [rad/s]
beta_dot_max = deg2rad(180);   % velocidad angular pitch [rad/s]

% Restricciones para etapas 1 a N-1
ocp.constraints.idxbx = 0:nx-1;
ocp.constraints.lbx = [-inf; -inf; -inf; deg2rad(-45); -3; -deg2rad(90); -inf];
ocp.constraints.ubx = [ inf;  inf;  inf; deg2rad(45);  3;  deg2rad(90);  inf];

% Estado inicial fijo (stage 0)
ocp.constraints.idxbx_0 = 0:nx-1;
ocp.constraints.lbx_0 = x0;
ocp.constraints.ubx_0 = x0;

%% ============================================
%% OPCIONES DEL SOLVER
%% ============================================
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


%% ============================================
%% CREAR SOLVER
%% ============================================
try
    ocp_solver = AcadosOcpSolver(ocp);
    fprintf('✓ Solver 2D creado exitosamente\n');
catch ME
    fprintf('✗ Error creando solver: %s\n', ME.message);
    rethrow(ME);
end

