function model = get_arm_model(varargin)
    % Modelo dinámico de un link rotacional actuado para MPC
    %
    % Estados:   [theta; theta_dot]
    % Control:   tau
    % Parámetros (tiempo-variante): [mass; r]
    %
    % Dinámica:
    %   theta_ddot = (tau + m*g*r*cos(theta)) / (m*r^2)

    import casadi.*

    %% ============================================
    %% PARÁMETROS FÍSICOS FIJOS
    %% ============================================
    g = 9.81;   % Gravedad [m/s^2]

    %% ============================================
    %% VARIABLES SIMBÓLICAS
    %% ============================================
    % Estados
    theta     = SX.sym('theta');      % Ángulo [rad]
    theta_dot = SX.sym('theta_dot');  % Velocidad angular [rad/s]
    states = vertcat(theta, theta_dot);

    % Control
    tau = SX.sym('tau');              % Torque [Nm]
    controls = tau;

    % Parámetros (tiempo-variante)
    mass = SX.sym('mass');            % Masa [kg]
    r    = SX.sym('r');               % Distancia al CoM [m]
    params = vertcat(mass, r);

    %% ============================================
    %% DINÁMICA CONTINUA
    %% ============================================
    theta_ddot = (tau + mass * g * r * cos(theta)) / (mass * r^2);

    states_dot = vertcat( ...
        theta_dot, ...
        theta_ddot ...
    );

    %% ============================================
    %% MODELO ACADOS
    %% ============================================
    model = AcadosModel();

    model.x = states;
    model.u = controls;
    model.p = params;
    model.xdot = SX.sym('xdot', 2, 1);

    % Forma explícita
    model.f_expl_expr = states_dot;

    % Forma implícita
    model.f_impl_expr = model.xdot - states_dot;

    model.name = 'arm_model';

end
