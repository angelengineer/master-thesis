% Crear modelo con parámetros
model = cart_pole_model_2();   % versión modificada
nx = length(model.x);
nu = length(model.u);
np = length(model.p);

% Crear MHE
mhe = AcadosOcpMhe();
mhe.model = model;

%% Horizonte y tiempo de muestreo
N = 10;                % número de pasos en la ventana
Ts = 0.1;              % tiempo de muestreo [s]
mhe.solver_options.N_horizon = N;
mhe.solver_options.tf = N * Ts;

%% Mediciones: se asume que medimos posición y ángulo
% Definir expresión de salida
p_sym = model.x(1); theta_sym = model.x(2);
y_expr = vertcat(p_sym, theta_sym);
mhe.model.y = y_expr;   % necesario para MHE

% Tamaño de la medición
ny = 2;

%% Covarianzas
% Ruido de medición (matriz V)
V = diag([0.01, 0.01]);   % desviaciones típicas: 0.1 m y 0.1 rad -> varianzas 0.01
mhe.cost.V = V;

% Ruido de proceso (para estados y parámetros)
% Se suele modelar que los parámetros varían lentamente con caminata aleatoria
Q = blkdiag( diag([1e-4, 1e-4, 1e-2, 1e-2]), ...  % estados: pequeña varianza
             diag([1e-6, 1e-8, 1e-8]) );           % parámetros: cambios muy lentos
mhe.cost.Q = Q;

% Coste de llegada (usamos la matriz de covarianza del estimado anterior)
% Esto requiere mantener una estimación previa y su covarianza. 
% Se puede inicializar con una matriz grande (poca confianza inicial)
mhe.cost.P0 = 1e3 * eye(nx+np);   % covarianza inicial

% Referencias para el coste de llegada: el estado estimado en el inicio de la ventana
mhe.cost.x0_ref = zeros(nx+np, 1);   % se actualizará en cada llamada

%% Restricciones (ejemplo: parámetros positivos)
mhe.constraints.idxbx = 0:nx+np-1;
mhe.constraints.lbx = [-inf; -inf; -inf; -inf; 0.1; 0.01; 1e-6];  % mínimos para m, L, I
mhe.constraints.ubx = [inf; inf; inf; inf; 5; 0.5; 1];            % máximos

%% Opciones del solver (similar a OCP)
mhe.solver_options.nlp_solver_type = 'SQP_RTI';
mhe.solver_options.integrator_type = 'GNSF';
% ... otras opciones

%% Crear solver
mhe_solver = AcadosOcpMheSolver(mhe);