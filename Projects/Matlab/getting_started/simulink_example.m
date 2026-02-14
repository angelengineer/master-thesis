%% Simulink example
clear all; clc;

%% Run minimal example
% get default simulink_opts
simulink_opts = get_acados_simulink_opts();
minimal_example_ocp;


%% Compile Sfunctions
cd c_generated_code

make_sfun; % ocp solver
make_sfun_sim; % integrator

cd ..
properties_cartpole;