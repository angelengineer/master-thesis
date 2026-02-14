function model = model_cart_pole(varargin)
    import casadi.*
    
    %% Parámetros del sistema
    M = 7.67;        % masa del carrito [kg]
    m = 0.7;         % masa del péndulo [kg]
    l = 0.06;        % longitud al CDG del péndulo [m]
    g = 9.81;        % gravedad [m/s^2]
    
    % Momento de inercia del péndulo
    I_pend_gmm2 = 3.975E+06;  % Iyy en g·mm²
    I_pend = I_pend_gmm2 * 1E-9;  % Conversión a kg·m²
    
    % Momento de inercia total sobre el pivote
    J = m*l^2 + I_pend;
    
    %% Variables simbólicas
    p = SX.sym('p');
    theta = SX.sym('theta');
    v = SX.sym('v');
    dtheta = SX.sym('dtheta');
    F = SX.sym('F');
    
    x = vertcat(p, theta, v, dtheta);
    xdot = SX.sym('xdot', 4, 1);
    u = F;
    
    %% Dinámica CORREGIDA
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    
    denom = (M + m)*J - (m*l*cos_theta)^2;
    
    % CORRECCIÓN 1: Signo del término gravitacional en p_ddot
    p_ddot = (J*F + m*l*sin_theta*(J*dtheta^2 + m*g*l*cos_theta)) / denom;
    
    % CORRECCIÓN 2: Signo del término centrífugo en theta_ddot
    theta_ddot = (-m*l*cos_theta*F - (M+m)*m*g*l*sin_theta ...
                  - (m*l)^2*sin_theta*cos_theta*dtheta^2) / denom;
    
    f_expl_expr = vertcat(v, dtheta, p_ddot, theta_ddot);
    f_impl_expr = f_expl_expr - xdot;
    
    %% Modelo
    model = AcadosModel();
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.f_expl_expr = f_expl_expr;
    model.f_impl_expr = f_impl_expr;
    model.name = 'cart_pole';
end