clear all; clc; close all;

%% Parámetros reales
M = 7.67;    % masa carro [kg]
L = 0.155;   % longitud [m]
g = 9.81;    % gravedad [m/s^2]
m_real = 0.4;        % ¡TU masa real!
I_real = m_real * L^2 * 0.33;  % inercia coherente

dt = 0.02;
T_sim = 10;
N = round(T_sim/dt);

%% Buffers
x_hist = zeros(4, N+1);
u_hist = zeros(1, N);
m_est_hist = zeros(1, N+1);
I_est_hist = zeros(1, N+1);

% Estado inicial
x_hist(:,1) = [0; pi; 0; 0];  % [p; theta; v; omega]
m_est_hist(1) = 0.3;  % estimación inicial arbitraria
I_est_hist(1) = 1e-4;

%% Inicializar RLS (lineal en [m; I])
theta_hat = [0.3; 1e-4];  % [m; I]
P = 1e2 * eye(2);         % covarianza inicial MODERADA (no 1e4)
lambda = 0.998;           % factor de olvido alto = estable

%% Simulación
fprintf('Paso |  tiempo |  theta [°] | omega [°/s] | m_real | m_est  |  error\n');
fprintf('-----------------------------------------------------------------------\n');

for k = 1:N
    t = (k-1)*dt;
    
    % --- 1. Generar control con excitación (para identificación) ---
    % Sin excitación, el sistema no "siente" la masa → RLS no converge
    u = 10 * sin(2*pi*0.8*t) + 5 * sin(2*pi*1.5*t);  % dos frecuencias para rica excitación
    u = max(-80, min(80, u));
    u_hist(k) = u;
    
    % --- 2. Simular planta REAL ---
    x_k = x_hist(:,k);
    x_next = simulate_plant(x_k, u, M, m_real, L, I_real, g, dt);
    x_hist(:,k+1) = x_next;
    
    % --- 3. Estimar aceleraciones (diferenciación central) ---
    if k >= 2
        % Aceleración angular alpha = d(omega)/dt
        alpha = (x_next(4) - x_hist(2, k-1)) / (2*dt);
        
        % Aceleración lineal a = d(v)/dt
        a = (x_next(3) - x_hist(1, k-1)) / (2*dt);
        
        % Variables actuales
        theta = x_k(2);
        omega = x_k(4);
        
        % --- 4. Regresor LINEAL para ecuación del péndulo ---
        % m * phi1 + I * phi2 = y  (donde y ≈ 0)
        phi1 = L^2 * alpha + g*L*sin(theta) + L*cos(theta)*a;
        phi2 = alpha;
        phi = [phi1, phi2];  % 1x2
        
        y = 0;  % idealmente cero (sin perturbaciones)
        
        % --- 5. Actualización RLS ESTÁNDAR ---
        denom = lambda + phi * P * phi';
        if abs(denom) > 1e-12
            K = (P * phi') / denom;
            theta_hat = theta_hat + K * (y - phi * theta_hat);
            P = (P - K * phi * P) / lambda;
        end
        
        % --- 6. Clamp físico suave ---
        theta_hat(1) = max(0.1, min(2.0, theta_hat(1)));  % masa razonable
        I_min = 0.05 * theta_hat(1) * L^2;
        I_max = 1.0 * theta_hat(1) * L^2;
        theta_hat(2) = max(I_min, min(I_max, theta_hat(2)));
        
        m_est_hist(k+1) = theta_hat(1);
        I_est_hist(k+1) = theta_hat(2);
    else
        m_est_hist(k+1) = m_est_hist(k);
        I_est_hist(k+1) = I_est_hist(k);
    end
    
    % --- Feedback cada 0.5s ---
    if mod(k, round(0.5/dt)) == 0
        err = abs(theta_hat(1) - m_real);
        fprintf('%4d | %6.2fs | %9.2f | %11.2f | %6.2f | %6.3f | %6.3f\n', ...
            k, t, rad2deg(x_next(2)), rad2deg(x_next(4)), m_real, theta_hat(1), err);
    end
    
    % --- Safety check ---
    if abs(x_next(2) - pi) > deg2rad(120)
        fprintf('\n⚠️  Péndulo caído en t=%.2fs\n', t);
        x_hist = x_hist(:,1:k+1);
        u_hist = u_hist(1:k);
        m_est_hist = m_est_hist(1:k+1);
        I_est_hist = I_est_hist(1:k+1);
        break;
    end
end

%% Resultados
fprintf('\n✅ Simulación completada\n');
fprintf('Masa real: %.3f kg | Masa final estimada: %.3f kg | Error: %.3f kg\n', ...
    m_real, theta_hat(1), abs(theta_hat(1) - m_real));

% Plot
t_vec = (0:length(m_est_hist)-1)' * dt;

figure('Position', [100, 100, 1000, 600]);

subplot(2,2,1);
plot(t_vec, rad2deg(x_hist(2,1:length(t_vec))), 'b-', 'LineWidth', 1.5);
hold on; yline(180, 'r--', 'Colgando', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Ángulo [deg]');
title('Ángulo del péndulo'); grid on;

subplot(2,2,2);
plot(t_vec, u_hist(1:length(t_vec)), 'r-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Fuerza [N]');
title('Control aplicado'); grid on;

subplot(2,2,3);
plot(t_vec, m_real * ones(size(t_vec)), 'g-', 'LineWidth', 2); hold on;
plot(t_vec, m_est_hist, 'b--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Masa [kg]');
title('Masa: real (verde) vs estimada (azul)');
legend('Real', 'Estimada', 'Location', 'best'); grid on;

subplot(2,2,4);
error_m = abs(m_real - m_est_hist);
plot(t_vec, error_m, 'm-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Error absoluto [kg]');
title('Error de estimación');
grid on;

sgtitle(sprintf('RLS CORREGIDO - Masa real: %.2f kg → Estimada: %.3f kg', m_real, theta_hat(1)), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% Funciones auxiliares
function x_next = simulate_plant(x, u, M, m, L, I, g, dt)
    p = x(1); th = x(2); v = x(3); w = x(4);
    s = sin(th); c = cos(th);
    
    M11 = M + m;
    M12 = m * L * c;
    M22 = m * L^2 + I;
    M_mat = [M11, M12; M12, M22];
    
    rhs = [u + m*L*s*w^2; -m*g*L*s];
    acc = M_mat \ rhs;
    
    % Euler explícito
    p_next = p + dt * v;
    th_next = th + dt * w;
    v_next = v + dt * acc(1);
    w_next = w + dt * acc(2);
    
    x_next = [p_next; th_next; v_next; w_next];
    x_next(2) = atan2(sin(x_next(2)), cos(x_next(2))); % normalizar ángulo
end