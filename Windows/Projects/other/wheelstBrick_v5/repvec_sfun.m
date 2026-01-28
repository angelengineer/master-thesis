function [sys, x0, str, ts] = repvec_sfun(t, x, u, flag, n)
% S-Function que repite un vector n veces

switch flag
    case 0  % Inicialización
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = -1;  % Tamaño dinámico
        sizes.NumInputs      = -1;  % Tamaño dinámico
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0  = [];
        str = [];
        ts  = [-1 0];
        
    case 3  % Salidas
        input_col = u(:);
        m = length(input_col);
        sys = zeros(m * n, 1);
        for i = 1:n
            sys((i-1)*m + 1 : i*m) = input_col;
        end
        
    case {1, 2, 4, 9}
        sys = [];
        
    otherwise
        error(['Flag no reconocido = ', num2str(flag)]);
end
end
