function model = get_cart_brick_model(varargin)
    import casadi.*
    
    nx = 4;
    nu = 1;
  
   
        M = 10; %cart mass
    
    % Brick properties
    %m = 6;          % mass [kg]
    lx = 0.3;       % length [m]
    ly = 0.8;       % width [m]
    lz = 1;       % height [m]
    g = 9.81;
    
    %% parámetros físicos (variables)
    m = SX.sym('m');   % masa del brick
    d = SX.sym('d');   % distancia al CoG
    params = vertcat(m, d);

        % Distance from joint to COM
    %d = lz/2;
    
    % Moment of inertia
    I_brick = (1/12) * m * (lx^2 + lz^2);  % Rotation about y-axis
    I = I_brick + m*d^2;  % Parallel axis theorem
    
    
    % State variables
    p = SX.sym('p');
    theta = SX.sym('theta');
    v = SX.sym('v');
    dtheta = SX.sym('dtheta');
    F = SX.sym('F');
    
    x = vertcat(p, theta, v, dtheta);
    xdot = SX.sym('xdot', nx, 1);
    u = F;
    
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    
    % Denominator
    denominator = (M + m) * I - (m*d*cos_theta)^2;
    
    % CORRECTED DYNAMICS
    dv_dt = ( I*F + I*m*d*sin_theta*dtheta^2 ...
              - m^2*d^2*g*sin_theta*cos_theta ) / denominator;
    
    dw_dt = ( -m*d*cos_theta*F - m^2*d^2*sin_theta*cos_theta*dtheta^2 ...
               +(M + m)*m*g*d*sin_theta ) / denominator;
    
    f_expl_expr = vertcat(v, dtheta, dv_dt, dw_dt);
    f_impl_expr = f_expl_expr - xdot;
    
 
    
    % Model
    model = AcadosModel();
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.p = params; %params

    model.f_expl_expr = f_expl_expr;
    model.f_impl_expr = f_impl_expr;
    model.name = 'cart_brick';
    
  
end