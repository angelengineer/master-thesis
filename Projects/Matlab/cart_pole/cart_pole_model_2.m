function model = cart_pole_model_2(varargin)
    import casadi.*
    % Valores por defecto (solo para inicialización, no se usan en las expresiones)
    M_nom = 7.67;   m_nom = 0.7;   L_nom = 0.155;   I_nom = 3.9750e-03;
    g = 9.81;
    % ... procesar argumentos para sobreescribir nominales (opcional)

    %% Variables simbólicas del estado
    p       = SX.sym('p');
    theta   = SX.sym('theta');
    v       = SX.sym('v');
    omega   = SX.sym('omega');
    x = vertcat(p, theta, v, omega);

    %% Parámetros a estimar
    m   = SX.sym('m');
    L   = SX.sym('L');
    I   = SX.sym('I');
    % Si también se quiere estimar M o g, se añaden aquí
    params = vertcat(m, L, I);   % vector de parámetros

    %% Trigonometría
    s = sin(theta); c = cos(theta);
    J = m * L^2 + I;

    %% Matriz de masa y rhs (usando los símbolos)
    M11 = M_nom + m;        % M_nom se mantiene fijo (carro)
    M12 = m * L * c;
    M22 = J;
    M_matrix = [[M11, M12]; [M12, M22]];
    rhs1 = F + m * L * s * omega^2;
    rhs2 = - m * g * L * s;
    rhs = [rhs1; rhs2];
    accel = M_matrix \ rhs;   % [a; alpha]
    a = accel(1); alpha = accel(2);

    %% Expresiones
    f_expl_expr = vertcat(v, omega, a, alpha);
    % ... (resto igual)

    %% Modelo Acados
    model = AcadosModel();
    model.x = x;
    model.u = F;
    model.p = params;          % <-- parámetros estimables
    model.f_expl_expr = f_expl_expr;
    model.name = 'cart_pole_est';
    % ... otros campos
end