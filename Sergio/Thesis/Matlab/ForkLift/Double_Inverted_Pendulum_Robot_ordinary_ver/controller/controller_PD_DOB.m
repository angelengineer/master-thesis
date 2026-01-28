%%% PD + DOB parameters %%%
%% Nominal parameter
I_bx = M_b*(L_by^2+L_bz^2)/12; %body main inertia in x[kgm^2]
I_by = M_b*(L_bx^2+L_by^2)/12; %body main inertia in y[kgm^2]
I_bz = M_b*(L_bz^2+L_bx^2)/12; %body main inertia in z[kgm^2]

I_mx = M_m*(L_my^2+L_mz^2)/12; %lift inertia in x[kgm^2]
I_my = M_m*(L_mx^2+L_my^2)/12; %lift inertia in y[kgm^2]
I_mz = M_m*(L_mz^2+L_mx^2)/12; %lift inertia in z[kgm^2]

I_wx = M_w*(R_wo^2+R_wi^2+W_w^2/3)/4; %wheel main inertia in x[kgm^2]
I_wy = M_w*(R_wo^2+R_wi^2)/2; %wheel main inertia in y[kgm^2]
I_wz = I_wx; %wheel main inertia in z[kgm^2]

I_fx = M_f*(L_fy^2+L_fz^2)/12; %fork main inertia in x[kgm^2]
I_fy = M_f*(L_fx^2+L_fy^2)/12; %fork main inertia in y[kgm^2]
I_fz = M_f*(L_fz^2+L_fx^2)/12; %fork main inertia in z[kgm^2]

% Taking into account the initial position of the lift
L_by_p = L_by;
L_by = L_mg;

M_n11 = (2*M_w+M_b+M_m+M_f)+R_wo^2+2*I_wy;
M_n21 = (L_bg*R_wo*cos(theta_pn))*M_b + (L_mg*R_wo*cos(theta_pn) +...
        R_wo*cos(theta_pn)*d_mn)*M_m +...
        (L_by*R_wo*cos(theta_pn) - L_fg*R_wo*sin(theta_fn + theta_pn) +...
        R_wo*cos(theta_pn)*d_mn)*M_f;
M_n22 = L_bg^2*M_b + (L_mg^2 + 2*L_mg*d_mn + d_mn^2)*M_m +...
        (L_fg^2 - 2*sin(theta_fn)*L_fg*L_by - 2*sin(theta_fn)*L_fg*d_mn +...
        L_by^2 + 2*L_by*d_mn + d_mn^2)*M_f + I_by + I_my + I_fy;
M_n33 = M_m + M_f;
M_n44 = L_fg^2*M_f + I_fy;

G_n3 = g*cos(theta_pn)*(M_f + M_m);

L_by = L_by_p;

%% Pitch angle
K_pp = 200; %pitch angle position gain[/s^2]
K_pd = 28.3; %pitch angle derivative gain[/s]28.3
g_spado = 2*pi*4; %cut-off angular frequency[rad/s]

%% Yaw angle
K_yp = 100; %yaw angle position gain[/s^2]
K_yd = 20; %yaw angle derivative gain[/s]
g_yaw = 2*pi*100; %cut-off angular frequency[rad/s]

%% Fork angle
K_fp = 100; %fork angle position gain[/s^2]
K_fd = 20; %fork angle derivative gain[/s]
g_fork = 2*pi*5; %cut-off angular frequency[rad/s]
g_reac = 2*pi*15; %cut-off angular frequency[rad/s]

%% Lift height
K_mp = 200; %lift height position gain[/s^2]
K_md = 50; %lift height derivative gain[/s]
g_lift = 2*pi*4; %cut-off angular frequency[rad/s]

%% Distance
K_wp = 0.07; %wheel angle position gain 0.6
K_wi = 0.00; %wheel angle integral gain 0.05
K_wd = 0.16; %wheel angle derivative gain 0.1

%% RCC
Ac=1;
omg=12;
gamm=1;
Kc=150;%2.5 82.5 4.381
Mc=Kc/omg^2;%0.04 3.23 0.137
Dc=gamm*2*sqrt(Kc*Mc);%0.625 32.6 1.549
g_ext = 2*pi*15; %cut-off angular frequency[rad/s]