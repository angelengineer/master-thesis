%%% Hardware parameters with modeling errors %%%
%% Body
M_b = 43.1; %body mass[kg]
L_bx = 0.20; %body depth[m]
L_by = 0.3325; %body height[m]
L_bz = 0.52; %body width[m]
D_b = M_b/(L_bx*L_by*L_bz); %body density[kg/m^3]
L_bg = L_by/2; %distance between COG of body and wheel axis[m]

%% Lift
M_m = 21.5; %body mass[kg]
L_mx = 0.10; %body depth[m]
L_my = 0.13; %body height[m]
L_mz = 0.42; %body width[m]
D_m = M_m/(L_mx*L_my*L_mz); %body density[kg/m^3]
L_mg = L_by+L_my/2; %distance between COG of lift and wheel axis[m]

%% Wheel
M_w = 1.21; %wheel mass[kg]
R_wo = 0.075; %wheel outer radius[m]
R_wi = 0.055; %wheel inner radius[m]
W_w = 0.025; %wheel width[m]
D_w = M_w/(pi*(R_wo^2-R_wi^2)*W_w); %wheel density[kg/m^3]
L_wt = (L_bz+W_w)/2; %half of tread[m]

%% Fork
M_f = 3.5; %fork mass[kg]
L_fx = 0.44; %fork length[m]
L_fy = 0.334; %fork width[m]
L_fz = 0.012; %fork thickness[m]
D_f = M_f/(L_fx*L_fy*L_fz); %fork density[kg/m^3]
L_fh = L_mg; %distance between bottom of body and fork[m]
L_fg = L_fx/2; %distance between COG of fork and fork axis[m]
