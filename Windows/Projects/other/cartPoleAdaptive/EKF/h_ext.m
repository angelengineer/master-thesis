function y = h_ext(x)
    % Medimos posición y velocidad
    % x = [pos; vel; theta; omega; Fext]
    y = [x(1);  % Posición del carro
         x(2)]; % Velocidad del carro
end