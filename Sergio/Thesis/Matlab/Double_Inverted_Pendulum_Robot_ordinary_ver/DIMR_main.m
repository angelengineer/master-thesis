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

%% Nominal conditions
theta_pn = 0.024; %nominal pitch angle[rad] -0.045
d_mn = 0; %nominal lift height[m]
theta_fn = -theta_pn; %nominal fork angle[rad] 0.045

%% Command generation
% Not in use
phi_cmd = 0*pi/180; %yaw angle cmd[rad]
theta_f_cmd = 0.037940459; %fork angle cmd[rad]
d_m_cmd = 0.12787808985686; %lift height cmd[m]
theta_p_cmd = -0.037940459; %pitch angle cmd[rad]
dis_cmd = 0;%3; %linear distance cmd[m]

% In use

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

%% Initial conditions
theta_p0 = theta_pn; %initial pitch angle[rad] -0.045
d_m0 = d_mn;%0.12787808985686; %initial lift height[m]
theta_f0 = -theta_p0; %initial fork angle[rad] 0.045
z_0 = R_wo+L_bmy+L_my/2; %Intial Z coordinate 0.4725

%% Planar trajectory generation
x_cmd = 3;%Desired position[m] 2.1 1.4
vel_cmd = 0.2; %Desired velocity[m/s] 0.3 0.2
acc_cmd = 0.05; %Desired acceleration[m/s^2] 0.15 0.05
jrk_cmd = 0.15; %Desired jerk[m/s^2] 0.3 0.15

tt = acc_cmd/jrk_cmd; %Transition time [s]
ta = (vel_cmd-jrk_cmd*tt^2)/acc_cmd; %Acceleration time [s]
tc = (x_cmd-2*(1/2*acc_cmd*ta^2+1/2*jrk_cmd*tt^2*ta+tt*vel_cmd))/vel_cmd; %Constant velocity time [s]

ts = 15; %Start time [s]
te = ts+2*ta+4*tt+tc;
ts2 = te+5;

%% Z trajectory generation
z_cmd = 0.9-z_0; %Desired height end-effector[m] 0.7
z_cmd2 = 0.9-0.7; %Desired height end-effector[m] 0.55

z_vel_cmd = 0.08; %Desired velocity[m/s]
z_acc_cmd = 0.05; %Desired acceleration[m/s^2]

taz = z_vel_cmd/z_acc_cmd; %Acceleration time [s]
tcz = (z_cmd-z_acc_cmd*taz^2)/z_vel_cmd; %Constant velocity time [s]
tcz2 = (z_cmd2-z_acc_cmd*taz^2)/z_vel_cmd; %Constant velocity time [s]

tsz = 10; %Start time [s]
tsz2 = ts2;

%% Run to initialize IK blocks
% cd(dir_cog);
% inverse_kinematics;
% cd(dir_current);

% cd(dir_cog);
% inverse;
% cd(dir_current);
