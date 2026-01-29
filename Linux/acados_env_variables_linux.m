% ===== acados env (Linux, MATLAB) =====

% rutas base
acados_dir = '/home/angel/Projects/acados';
casadi_dir = '/home/angel/Projects/acados/external/casadi-matlab';

% interfaz MATLAB de acados
matlab_interface_dir = fullfile(acados_dir, 'interfaces', 'acados_matlab_octave');

% paths MATLAB
addpath(matlab_interface_dir);
addpath(casadi_dir);

% variables de entorno
setenv('ACADOS_INSTALL_DIR', acados_dir);
setenv('ENV_RUN', 'true');

%% === CONFIGURACIÓN LD_LIBRARY_PATH ===
% Construir path dinámicamente incluyendo TODAS las carpetas c_generated_code_*

% Path base existente
ld_path = getenv('LD_LIBRARY_PATH');

% 1. Añadir librerías acados
ld_path = [ld_path ':' fullfile(acados_dir, 'lib')];

% 2. Añadir directorio actual (pwd)
ld_path = [ld_path ':' pwd];

% 3. Buscar y añadir TODAS las carpetas c_generated_code_*
gen_pattern = fullfile(pwd, 'c_generated_code_*');
gen_dirs = dir(gen_pattern);

for k = 1:length(gen_dirs)
    if gen_dirs(k).isdir && ~startsWith(gen_dirs(k).name, '.')
        full_path = fullfile(pwd, gen_dirs(k).name);
        ld_path = [ld_path ':' full_path];
        fprintf('Añadido al LD_LIBRARY_PATH: %s\n', gen_dirs(k).name);
    end
end

% Aplicar
setenv('LD_LIBRARY_PATH', ld_path);

disp('acados + CasADi ready (Linux)');
disp(['LD_LIBRARY_PATH actualizado con ' num2str(length(gen_dirs)) ' directorios de código generado']);