%% Simulink example
clear all; clc;clear mex; rehash;

%% Run minimal example
% get default simulink_opts
simulink_opts = get_acados_simulink_opts();
cart_pole_ocp;


%% Compile Sfunctions
cd c_generated_code

make_sfun; % ocp solver
make_sfun_sim; % integrator

cd ..
cart_pole_properties;
addpath(genpath(pwd))