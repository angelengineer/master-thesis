
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
x0 = [0; 0; 0; 0]; % initial state



%% model dynamics
model = get_cart_brick_model();
nx = length(model.x); % state size
nu = length(model.u); % input size

%% OCP formulation object
ocp = AcadosOcp();
ocp.model = model;

%% cost in nonlinear least squares form
W_x = diag([1e-3, 1e3, 1e3, 1e-2]);

W_x_nom = diag([100, 100, 1, 1]);   % seguimiento normal
W_x_soft = diag([1, 1, 1e-3, 1e-3]); % cuando alguien empuja


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
W_nom_full  = blkdiag(W_x, W_u);
W_soft_full = blkdiag(W_x, W_u);

ocp.cost.W = blkdiag(W_x, W_u);
ocp.cost.yref = zeros(ny, 1);
ocp.model.cost_y_expr = vertcat(model.x, model.u);

% terminal cost term
ny_e = nx;
ocp.cost.cost_type_e = 'NONLINEAR_LS';




ocp.model.cost_y_expr_e = model.x;
ocp.cost.yref_e = zeros(ny_e, 1);
ocp.cost.W_e = W_x;

U_max = 80;

% 1. Usar 'BGH' (Bounds + General constraints expressed via h) o 'AUTO'
ocp.constraints.constr_type = 'AUTO';  % O usa 'AUTO' como en el ejemplo exitoso

% 2. Restricciones de CONTROL como caja
ocp.constraints.idxbu = 0;        % Índice 0 = control u
ocp.constraints.lbu = -U_max;     % Límite inferior
ocp.constraints.ubu = U_max;      % Límite superior
% 
% 3. Restricciones de ESTADO como caja
ocp.constraints.idxbx = 0;        % Índice 0 = estado 1 (posición p)
ocp.constraints.lbx = -5;         % Límite inferior para posición
ocp.constraints.ubx = 5;          % Límite superior para posición

ocp.constraints.idxbx_e = 0;      % Terminal
ocp.constraints.lbx_e = -5;
ocp.constraints.ubx_e = 5;

ocp.constraints.idxbx_0 = 0;      % Inicial
ocp.constraints.lbx_0 = -5;
ocp.constraints.ubx_0 = 5;

ocp.constraints.x0 = x0;
% only bound on u on initial stage and path
% ocp.model.con_h_expr = model.u;
% ocp.model.con_h_expr_0 = model.u;
% 
% U_max = 80;
% ocp.constraints.lh = -U_max;
% ocp.constraints.lh_0 = -U_max;
% ocp.constraints.uh = U_max;
% ocp.constraints.uh_0 = U_max;

% % RESTRICCIONES DE ESTADO - AÑADE ESTO:
% % Define límites para los estados (x)
% % Ejemplo: -5 < posición < 5, -π < ángulo < π, etc.
% 
% % Para todos los estados en etapas 1 a N-1
% lim = 5000;
% ocp.constraints.idxbx = 0;  % Índices de estados a restringir (todos)
% ocp.constraints.lbx = -lim;   % Límites inferiores
% ocp.constraints.ubx = lim;      % Límites superiores
% 
% % Para el estado terminal (etapa N)
% ocp.constraints.idxbx_e = 0;  % Restringir todos los estados terminales
% ocp.constraints.lbx_e = -lim;   % Límites inferiores terminales
% ocp.constraints.ubx_e = lim;      % Límites superiores terminales
% 
% % Para el estado inicial (etapa 0) - opcional
% ocp.constraints.idxbx_0 = 0;  % Restringir estado inicial
% ocp.constraints.lbx_0 = -lim;
% ocp.constraints.ubx_0 = lim;
% 

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

% create solver
ocp_solver = AcadosOcpSolver(ocp);

% solver initial guess
x_traj_init = zeros(nx, N+1);
u_traj_init = zeros(nu, N);

%% call ocp solver
% update initial state
ocp_solver.set('constr_x0', x0);

% set trajectory initialization
ocp_solver.set('init_x', x_traj_init); % states
ocp_solver.set('init_u', u_traj_init); % inputs
ocp_solver.set('init_pi', zeros(nx, N)); % multipliers for dynamics equality constraints

% change values for specific shooting node using:
%   ocp_solver.set('field', value, optional: stage_index)





% solve
ocp_solver.solve();
% get solution
utraj = ocp_solver.get('u');
xtraj = ocp_solver.get('x');

status = ocp_solver.get('status'); % 0 - success
assert(status == 0, sprintf('solver failed with status %d', status))
ocp_solver.print('stat')

%% plots
ts = linspace(0, T, N+1);
figure; hold on;
states = {'p', 'theta', 'v', 'dtheta'};
for i=1:length(states)
    subplot(length(states), 1, i);
    plot(ts, xtraj(i,:)); grid on;
    ylabel(states{i});
    xlabel('t [s]')
end

figure
stairs(ts, [utraj'; utraj(end)])
ylabel('F [N]')
xlabel('t [s]')
grid on
