classdef MassRLSEstimator
    % Estimador RLS para masa del péndulo usando error de predicción
    % Método: linearización del modelo alrededor de θ̂_k
    
    properties (Access = private)
        theta_hat      % [m; I] estimado actual
        P              % Covarianza de estimación
        lambda         % Factor de olvido (0.98-0.999)
        L              % Longitud del péndulo (fija, medida)
        M              % Masa del carro (conocida)
        g              % Gravedad
        dt             % Paso de tiempo de muestreo
        min_mass       % Masa mínima física (evita valores no realistas)
        max_mass       % Masa máxima física
    end
    
    properties (Access = public)
        history_mass   % Historial de estimaciones para plotting
        history_I      % Historial de inercia estimada
        timestamps     % Timestamps de cada actualización
    end
    
    methods
        function obj = MassRLSEstimator(m0, I0, L_val, M_val, dt_val)
            % Constructor
            % m0, I0: valores iniciales (pueden ser arbitrarios)
            % L_val: longitud medida del péndulo [m]
            % M_val: masa conocida del carro [kg]
            % dt_val: paso de tiempo de control [s]
            
            obj.theta_hat = [m0; I0];
            obj.P = 1e4 * eye(2);          % Covarianza inicial alta = alta incertidumbre
            obj.lambda = 0.995;            % Factor de olvido (0.99 = memoria larga)
            obj.L = L_val;
            obj.M = M_val;
            obj.g = 9.81;
            obj.dt = dt_val;
            obj.min_mass = 0.1;            % kg mínimo físico
            obj.max_mass = 5.0;            % kg máximo razonable
            
            obj.history_mass = m0;
            obj.history_I = I0;
            obj.timestamps = 0;
        end
        
        function [m_est, I_est] = update(obj, x_k, u_k, x_next_meas, t_now)
            % Actualiza estimación usando:
            %   x_k        : estado actual [p; theta; v; omega]
            %   u_k        : control aplicado
            %   x_next_meas: estado medido en siguiente paso
            %   t_now      : timestamp actual
            
            % 1. Predicción con modelo usando parámetros actuales
            x_pred = obj.predict_state(x_k, u_k, obj.theta_hat(1), obj.theta_hat(2));
            
            % 2. Error de predicción (solo estados críticos: theta y omega)
            %    Ignoramos posición del carro (menos sensible a masa)
            e = x_next_meas([2;4]) - x_pred([2;4]);  % [theta_err; omega_err]
            
            % 3. Calcular gradiente del modelo wrt parámetros (regresor φ)
            %    φ = ∂f/∂θ evaluado en θ̂_k
            phi = obj.compute_gradient(x_k, u_k, obj.theta_hat(1), obj.theta_hat(2));
            
            % 4. Actualización RLS estándar
            %    K = P * φ' / (λ + φ * P * φ')
            %    θ̂_{k+1} = θ̂_k + K * (e - φ * (θ̂_k - θ̂_k)) = θ̂_k + K * e
            denom = obj.lambda + phi * obj.P * phi';
            if denom < 1e-12
                % Evitar división por cero numérica
                return;
            end
            
            K = (obj.P * phi') / denom;
            theta_update = K * e;
            obj.theta_hat = obj.theta_hat + theta_update;
            
            % 5. Actualizar covarianza
            obj.P = (obj.P - K * phi * obj.P) / obj.lambda;
            
            % 6. Clamp físico (masa no puede ser negativa ni absurdamente grande)
            obj.theta_hat(1) = max(obj.min_mass, min(obj.max_mass, obj.theta_hat(1)));
            % Inercia coherente con masa: I ≈ m*L^2 * k (k ~ 0.3-1.0 para formas comunes)
            I_min = 0.1 * obj.theta_hat(1) * obj.L^2;
            I_max = 2.0 * obj.theta_hat(1) * obj.L^2;
            obj.theta_hat(2) = max(I_min, min(I_max, obj.theta_hat(2)));
            
            % 7. Guardar historial
            obj.history_mass = [obj.history_mass; obj.theta_hat(1)];
            obj.history_I = [obj.history_I; obj.theta_hat(2)];
            obj.timestamps = [obj.timestamps; t_now];
            
            m_est = obj.theta_hat(1);
            I_est = obj.theta_hat(2);
        end
        
        function x_next = predict_state(obj, x_k, u_k, m_val, I_val)
            % Predice siguiente estado usando Euler explícito
            % x_k = [p; theta; v; omega]
            
            p = x_k(1); theta = x_k(2); v = x_k(3); omega = x_k(4);
            F = u_k;
            
            s = sin(theta);
            c = cos(theta);
            
            % Matriz de masa con parámetros actuales
            M11 = obj.M + m_val;
            M12 = m_val * obj.L * c;
            M22 = m_val * obj.L^2 + I_val;
            M_mat = [M11, M12; M12, M22];
            
            % Vector de fuerzas
            rhs1 = F + m_val * obj.L * s * omega^2;
            rhs2 = -m_val * obj.g * obj.L * s;
            rhs = [rhs1; rhs2];
            
            % Aceleraciones
            accel = M_mat \ rhs;
            a = accel(1);
            alpha = accel(2);
            
            % Integración Euler
            p_next = p + obj.dt * v;
            theta_next = theta + obj.dt * omega;
            v_next = v + obj.dt * a;
            omega_next = omega + obj.dt * alpha;
            
            x_next = [p_next; theta_next; v_next; omega_next];
        end
        
        function phi = compute_gradient(obj, x_k, u_k, m_val, I_val)
            % Calcula gradiente ∂f/∂[m; I] para el regresor RLS
            % Aproximación numérica (más robusta que derivada analítica)
            
            eps_m = 1e-4;   % perturbación para masa
            eps_I = 1e-6;   % perturbación para inercia
            
            % Predicción nominal
            x_nom = obj.predict_state(x_k, u_k, m_val, I_val);
            
            % Predicción con m + ε
            x_m_pert = obj.predict_state(x_k, u_k, m_val + eps_m, I_val);
            df_dm = (x_m_pert([2;4]) - x_nom([2;4])) / eps_m;  % solo theta, omega
            
            % Predicción con I + ε
            x_I_pert = obj.predict_state(x_k, u_k, m_val, I_val + eps_I);
            df_dI = (x_I_pert([2;4]) - x_nom([2;4])) / eps_I;
            
            % Regresor 2x2: [∂theta/∂m, ∂theta/∂I; ∂omega/∂m, ∂omega/∂I]
            phi = [df_dm, df_dI];
        end
        
        function plot_estimation(obj)
            % Visualiza convergencia de la estimación
            figure('Name', 'RLS Mass Estimation', 'NumberTitle', 'off');
            
            subplot(2,1,1);
            plot(obj.timestamps, obj.history_mass, 'b-', 'LineWidth', 2);
            xlabel('Time [s]'); ylabel('Estimated mass [kg]');
            grid on; title('Pendulum mass estimation');
            
            subplot(2,1,2);
            plot(obj.timestamps, obj.history_I, 'r-', 'LineWidth', 2);
            xlabel('Time [s]'); ylabel('Estimated inertia [kg·m^2]');
            grid on; title('Pendulum inertia estimation');
        end
    end
end