%%% This is a m-file for DIMR.slx. %%%
%%% You have to run it in "/Double_Inverted_Pendulum_Robot_ordinary_ver.%%%
%% incantation
close all;
clear;

%% Path of directories
dir_current = cd;
dir_model = fullfile(dir_current,"model");
dir_control = fullfile(dir_current,"controller");
dir_cog = fullfile(dir_current,"CoG");

%% Initial and nominal conditions
theta_p0 = 0*pi/180; %initial pitch angle[rad]
d_m0 = 0.12787808985686; %initial lift height[m]
theta_f0 = 0; %initial fork angle[rad]

theta_pn = 0*pi/180; %nominal pitch angle[rad]
d_mn = 0.00; %nominal lift height[m]
theta_fn = 0; %nominal fork angle[rad]

%% Command generation
% Not in use
phi_cmd = 0*pi/180; %yaw angle cmd[rad]
theta_f_cmd = 0.037940459; %fork angle cmd[rad]

% In use
d_m_cmd = 0.12787808985686; %lift height cmd[m]
theta_p_cmd = -0.037940459; %pitch angle cmd[rad]
dis_cmd = 3;%3; %linear distance cmd[m]

%% Load robot configuration
cd(dir_model);
model_params_with_errors;
model_params_without_errors;
model_params_env;
cd(dir_current);

cd(dir_control);
controller_init;
controller_PD_DOB
cd(dir_current);

cd(dir_cog);
controller_init;
controller_PD_DOB
cd(dir_current);
