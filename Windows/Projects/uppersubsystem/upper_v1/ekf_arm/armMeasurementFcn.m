function y = armMeasurementFcn(x)
    % Medimos solo el ángulo theta
    % x = [theta; theta_dot; r]
    y = x(1);  % Ángulo medido [rad]
end