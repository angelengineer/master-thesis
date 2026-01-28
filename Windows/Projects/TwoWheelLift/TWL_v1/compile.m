%% Simulink example
%
clear all; clc; clear mex;
fprintf("Building sfunctions...");
if isfolder("c_generated_code")
    rmdir("c_generated_code", "s");   
end
if isfolder("build")
    rmdir("build", "s");   
end


%% Run minimal example
% get default simulink_opts
simulink_opts = get_acados_simulink_opts;
solver;


%% Compile Sfunctions
cd c_generated_code

make_sfun; % ocp solver
make_sfun_sim; % integrator
cd ..

robotParams;
