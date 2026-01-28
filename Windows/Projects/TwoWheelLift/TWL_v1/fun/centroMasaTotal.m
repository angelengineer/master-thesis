function com_total = centroMasaTotal(data)
% centroMasaTotal Calcula el centro de masa total
%
% Entrada:
%   data : matriz Nx4
%          [masa_i, x_i, y_i, z_i]
%
% Salida:
%   com_total : vector 1x3 [X, Y, Z]

    masas = data(:,1);
    posiciones = data(:,2:4);

    masa_total = sum(masas);

    if masa_total == 0
        error('La masa total no puede ser cero');
    end

    com_total = sum(masas .* posiciones, 1) / masa_total;
end
