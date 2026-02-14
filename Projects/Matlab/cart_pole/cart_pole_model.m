function model = cart_pole_model(varargin)


    import casadi.*

    %% Valores por defecto (hardcodeados)
    M  = 7.67;      % masa carro [kg]
    m  = 0.7;       % masa péndulo [kg]
    L  = 0.155;     % distancia pivote → CM [m]
    I  = 3.9750e-03;         % momento de inercia respecto al CM [kg*m^2]
    dz = 0;         % offset vertical del pivote [m]  (no afecta)
    dy = 0;         % offset lateral del pivote [m]   (no afecta)
    g  = 9.81;      % gravedad [m/s^2]
    delta_t = [];

    %% Procesar argumentos entrada (pares nombre-valor)
    if nargin > 0
        if mod(nargin,2) ~= 0
            error('Los argumentos deben ser pares nombre-valor.');
        end
        for i = 1:2:nargin
            name = varargin{i};
            val  = varargin{i+1};
            switch lower(name)
                case 'm',       m = val;
                case 'M',       M = val;
                case 'l',       L = val;
                case 'i',       I = val;
                case 'dz',      dz = val;
                case 'dy',      dy = val;
                case 'g',       g = val;
                case 'delta_t', delta_t = val;
                otherwise
                    warning('Parámetro "%s" desconocido, se ignora.', name);
            end
        end
    end

    %% Momento de inercia respecto al pivote (valor numérico fijo)
    J = m * L^2 + I;   % J es un número (double)

    %% Variables simbólicas del estado
    p       = SX.sym('p');       % posición carro [m]
    theta   = SX.sym('theta');   % ángulo péndulo (0 = hacia abajo) [rad]
    v       = SX.sym('v');       % velocidad carro [m/s]
    omega   = SX.sym('omega');   % velocidad angular [rad/s]
    x = vertcat(p, theta, v, omega);
    nx = 4;

    % Derivadas del estado
    xdot = SX.sym('xdot', nx, 1);

    % Entrada de control
    F = SX.sym('F');
    u = F;
    nu = 1;

    %% Trigonometría
    s = sin(theta);
    c = cos(theta);

    %% Matriz de masa y vector de fuerzas (valores numéricos insertados directamente)
    %  Sistema M * [a; alpha] = rhs, con a = dv/dt, alpha = d(omega)/dt
    M11 = M + m;
    M12 = m * L * c;
    M22 = J;
    M_matrix = [[M11, M12]; [M12, M22]];

    rhs1 = F + m * L * s * omega^2;
    rhs2 = - m * g * L * s;
    rhs = [rhs1; rhs2];

    %% Resolver sistema lineal para aceleraciones
    accel = M_matrix \ rhs;   % [a; alpha]
    a     = accel(1);
    alpha = accel(2);

    %% Dinámica continua: forma explícita e implícita
    f_expl_expr = vertcat(v, omega, a, alpha);
    f_impl_expr = f_expl_expr - xdot;

    %% Dinámica discreta (Euler explícito) si se proporciona delta_t
    if ~isempty(delta_t)
        disc_dyn_expr = x + delta_t * f_expl_expr;
    else
        disc_dyn_expr = [];
    end

    %% Crear estructura AcadosModel (sin parámetros simbólicos)
    model = AcadosModel();
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.z = [];          % no hay variables algebraicas
    model.p = [];          % sin parámetros (todo hardcodeado)
    model.f_expl_expr = f_expl_expr;
    model.f_impl_expr = f_impl_expr;
    model.disc_dyn_expr = disc_dyn_expr;
    model.name = 'cart_pole';
end