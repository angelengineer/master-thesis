function model = cart_pole_adaptive_model(varargin)
    import casadi.*
    
    %% Parámetros SIMBÓLICOS (crítico para adaptación online)
    M_sym = SX.sym('M');   % masa carro [kg]
    m_sym = SX.sym('m');   % masa péndulo [kg] - ESTIMAR ONLINE
    L_sym = SX.sym('L');   % longitud [m]
    I_sym = SX.sym('I');   % inercia [kg*m^2]
    g_sym = SX.sym('g');   % gravedad [m/s^2]
    
    %% Valores por defecto (solo para documentación - NO se usan aquí)
    M_def = 7.67; 
    m_def = 0.7;
    L_def = 0.155; 
    I_def = 3.9750e-03; 
    g_def = 9.81;
    
    %% Procesar argumentos (opcional)
    if nargin > 0
        if mod(nargin,2) ~= 0
            error('Los argumentos deben ser pares nombre-valor.');
        end
        for i = 1:2:nargin
            name = lower(varargin{i});
            val  = varargin{i+1};
            switch name
                case 'm',  m_def = val;
                case 'M',  M_def = val;
                case 'l',  L_def = val;
                case 'i',  I_def = val;
                case 'g',  g_def = val;
            end
        end
    end
    
    %% Variables de estado y control
    p     = SX.sym('p');       % posición carro [m]
    theta = SX.sym('theta');   % ángulo péndulo [rad]
    v     = SX.sym('v');       % velocidad carro [m/s]
    omega = SX.sym('omega');   % velocidad angular [rad/s]
    x = vertcat(p, theta, v, omega);
    nx = 4;
    
    xdot = SX.sym('xdot', nx, 1);
    F = SX.sym('F');           % fuerza en el carro [N]
    u = F;
    nu = 1;
    
    %% Dinámica con parámetros simbólicos
    s = sin(theta);
    c = cos(theta);
    
    % Matriz de masa
    M11 = M_sym + m_sym;
    M12 = m_sym * L_sym * c;
    M22 = m_sym * L_sym^2 + I_sym;  % J = m*L^2 + I
    M_matrix = [M11, M12; M12, M22];
    
    % Vector de fuerzas
    rhs1 = F + m_sym * L_sym * s * omega^2;
    rhs2 = -m_sym * g_sym * L_sym * s;
    rhs = [rhs1; rhs2];
    
    % Resolver para aceleraciones
    accel = M_matrix \ rhs;
    a = accel(1);
    alpha = accel(2);
    
    % Dinámica explícita
    f_expl_expr = vertcat(v, omega, a, alpha);
    f_impl_expr = f_expl_expr - xdot;
    
    %% Crear modelo Acados CON parámetros simbólicos
    model = AcadosModel();
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.p = vertcat(M_sym, m_sym, L_sym, I_sym, g_sym);  % ← PARÁMETROS SIMBÓLICOS
    model.f_expl_expr = f_expl_expr;
    model.f_impl_expr = f_impl_expr;
    model.name = 'cart_pole_adaptive';
    
    % NOTA: NO hay model.p_val - los valores se pasan vía ocp.parameter_values
end