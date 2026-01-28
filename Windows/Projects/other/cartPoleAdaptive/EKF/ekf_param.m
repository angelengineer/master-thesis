%% PARÁMETROS DEL EKF
% Covarianza del proceso - AJUSTA ESTOS VALORES
Q = diag([0.001,  % incertidumbre posición (m^2)
          0.01,   % incertidumbre velocidad (m^2/s^2)
          0.001,  % incertidumbre ángulo (rad^2)
          0.01,   % incertidumbre velocidad angular (rad^2/s^2)
          1.0]);  % incertidumbre fuerza externa (N^2) - CLAVE

% Covarianza de medición - AJUSTA
R = diag([0.0001,  % ruido posición (m^2)
          0.001]); % ruido velocidad (m^2/s^2)

% Covarianza inicial
P0 = diag([0.1, 0.1, 0.1, 0.1, 10]);

% Umbral de detección
umbral_Fext = 5;  % [N] - fuerza que consideramos "colisión"

% Parámetros físicos (DEBEN COINCIDIR con tu planta)
M = 0.5;    % kg
m = 0.2;    % kg  
l = 0.3;    % m
g = 9.81;   % m/s^2
b = 0.1;    % N/(m/s)