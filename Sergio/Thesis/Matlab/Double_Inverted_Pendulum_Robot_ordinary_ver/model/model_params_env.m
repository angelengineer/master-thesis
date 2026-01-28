%%% Environment parameters %%%
%% Ground
Angle_slope = 0; %angle of ground[deg]
Stiffness = 1e9; %stiffness of ground[N/m]
Damping = 1e6; %viscosity of ground[N/(m/s)]
Transition_region_width = 1e-4; %boundary of contact[m]
L_ex = 100; %ground length[m]
L_ey = 100; %ground width[m]
L_ez = 0.01; %ground thickness[m]

%% Load
M_l = 5.0; %load mass[kg]
L_lx = 0.1;
L_ly = 0.1;
L_lz = 0.1;
D_l = M_l/(L_lx*L_ly*L_lz); %load density[kg/m^3] 

%% Environment
g = 9.80665; % m/s^2