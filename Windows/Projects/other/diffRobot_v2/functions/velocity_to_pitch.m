%% CONTROLADOR DE ALTO NIVEL: velocity_to_pitch.m
function [beta_ref, tau_diff_ref] = velocity_to_pitch(v_desired, omega_desired, current_state)
    % Convierte comandos de velocidad a referencia de pitch y diferencia de torque
    % Inputs:
    %   v_desired: velocidad lineal deseada [m/s]
    %   omega_desired: velocidad angular deseada [rad/s]
    %   current_state: [β, β_dot, v, ω]
    %   params: parámetros del controlador
    
    % Parámetros del PI para velocidad
    Kp_v = 0.15;   % Ganancia proporcional para velocidad
    Ki_v = 0.02;   % Ganancia integral para velocidad
    max_beta_ref = deg2rad(15);  % Máxima inclinación permitida
    
    % Estado actual
    v_actual = current_state(3);
    omega_actual = current_state(4);
    
    % Persistente para integral
    persistent v_error_integral;
    if isempty(v_error_integral)
        v_error_integral = 0;
    end
    
    % 1. Control de velocidad -> Referencia de pitch
    v_error = v_desired - v_actual;
    
    % Actualizar integral con anti-windup
    v_error_integral = v_error_integral + v_error;
    
    % Limitar integral para evitar windup
    max_integral = max_beta_ref / Ki_v;
    v_error_integral = max(min(v_error_integral, max_integral), -max_integral);
    
    % Calcular referencia de pitch
    beta_ref = Kp_v * v_error + Ki_v * v_error_integral;
    
    % Limitar referencia de pitch
    beta_ref = max(min(beta_ref, max_beta_ref), -max_beta_ref);
    
    % 2. Control de giro -> Diferencia de torque
    K_omega = 2.0;  % Ganancia para giro
    omega_error = omega_desired - omega_actual;
    tau_diff_ref = K_omega * omega_error;
    
    % Limitar diferencia de torque
    max_tau_diff = 8.0;  % Nm
    tau_diff_ref = max(min(tau_diff_ref, max_tau_diff), -max_tau_diff);
end