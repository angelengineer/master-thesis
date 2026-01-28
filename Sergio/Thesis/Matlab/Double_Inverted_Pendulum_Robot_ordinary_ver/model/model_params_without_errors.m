%%% Hardware parameters without modeling errors %%%
%% Simulation cycle
T_plant = 1e-6; %simulation cycle of robot[s]

%% Gear ratio
N_w = 11; %wheel gear ratio
N_f =33; %fork gear ratio

%% Torque limit
Torque_limit_w = 1.27; %wheel motor torque limit[Nm]
Torque_limit_f = 1.3; %fork motor torque limit[Nm]
Torque_limit_l = 1.3; %lift motor torque limit[Nm]
%% Motor Internal mechanics
K_stiff = 0;
B_damp = 0;