function model = get_wheeled_brick_model()
    import casadi.*

    %% Parámetros
    m_b = 16;
    m_w = 1;
    R   = 0.2;
    d   = 0.5;
    g   = 9.81;

    ancho = 0.8;
    alto  = 1.0;

    I_b = (1/12)*m_b*(ancho^2 + alto^2);
    I_w = 0.5*m_w*R^2;

    M = m_b + 2*m_w + 2*I_w/R^2;
    C = m_b*d;
    I = m_b*d^2 + I_b;

    %% Variables
    beta     = SX.sym('beta');
    beta_dot = SX.sym('beta_dot');
    u_s      = SX.sym('u_s');

    %% Dinámica reducida
    cosb = cos(beta);
    sinb = sin(beta);

    denom = I - (C^2/M)*cosb^2;

    beta_ddot = ( ...
        m_b*g*d*sinb ...
      - (C*cosb/M)*(u_s/R + C*sinb*beta_dot^2) ...
      ) / denom;

    %% Estados
    x = [beta; beta_dot];
    xdot = [beta_dot; beta_ddot];

    %% Modelo acados
    model = AcadosModel();
    model.x = x;
    model.xdot = SX.sym('xdot',2,1);
    model.u = u_s;
    model.f_expl_expr = xdot;
    model.f_impl_expr = xdot - model.xdot;
    model.name = 'wheeled_brick_beta_only';
end
