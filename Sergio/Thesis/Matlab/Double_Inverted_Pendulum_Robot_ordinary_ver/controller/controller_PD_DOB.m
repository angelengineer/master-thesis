%%% PD + DOB parameters %%%
%% Nominal parameter
I_bmx = M_bm*(L_bmy^2+L_bmz^2)/12; %main body inertia in x[kgm^2]
I_bmy = M_bm*(L_bmx^2+L_bmy^2)/12; %main body inertia in y[kgm^2]
I_bmz = M_bm*(L_bmz^2+L_bmx^2)/12; %main body inertia in z[kgm^2]

I_bsx = M_bs*(L_bsy^2+L_bsz^2)/12; %secondary body inertia in x[kgm^2]
I_bsy = M_bs*(L_bsx^2+L_bsy^2)/12; %secondary body inertia in y[kgm^2]
I_bsz = M_bs*(L_bsz^2+L_bsx^2)/12; %secondary body inertia in z[kgm^2]

C_bx = M_bs*(L_bmx/2+L_bsx/2)/(M_bm+M_bs); %total body CoG position in x
C_by = 0; %total body CoG position in y
C_bz = (M_bm*(L_bmy/2)+M_bs*(L_bmy+L_bsy/2-0.129))/(M_bm+M_bs); %total body CoG position in z

I_bx = I_bmx+M_bm*(C_bz-L_bmy/2)^2+I_bsx+M_bs*(L_bmy+L_bsy/2-0.129-C_bz)^2; %total body inertia in x[kgm^2]
I_by = I_bmy+M_bm*((C_bz-L_bmy/2)^2+C_bx^2)+I_bsy+M_bs*((L_bmy+L_bsy/2-0.129-C_bz)^2+(L_bmx/2+L_bsx/2-C_bx)^2); %total body inertia in y[kgm^2]
I_bz = I_bmz+M_bm*C_bx^2+I_bsz+M_bs*(L_bmx/2+L_bsx/2-C_bx)^2; %total body inertia in z[kgm^2]

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
% dynamics;

dynamics;

M_n11 = M(1,1);
M_n21 = M(2,1);
M_n22 = M(2,2);
M_n33 = M(3,3);
M_n44 = M(4,4);

% M_n11 = (2*M_w+M_b+M_m+M_f)*R_wo^2+2*I_wy;
% M_n21 = (L_bg*R_wo*cos(theta_pn))*M_b + (L_mg*R_wo*cos(theta_pn) +...
%         R_wo*cos(theta_pn)*d_mn)*M_m +...
%         (L_by*R_wo*cos(theta_pn) - L_fg*R_wo*sin(theta_fn + theta_pn) +...
%         R_wo*cos(theta_pn)*d_mn)*M_f;
% M_n22 = L_bg^2*M_b + (L_mg^2 + 2*L_mg*d_mn + d_mn^2)*M_m +...
%         (L_fg^2 - 2*sin(theta_fn)*L_fg*L_by - 2*sin(theta_fn)*L_fg*d_mn +...
%         L_by^2 + 2*L_by*d_mn + d_mn^2)*M_f + I_by + I_my + I_fy;
% M_n33 = M_m + M_f;
% M_n44 = L_fg^2*M_f + I_fy;
% 
% G_n3 = g*cos(theta_pn)*(M_f + M_m);

%% Pitch angle
K_pp = 200; %pitch angle position gain[/s^2] 200
K_pd = 50; %pitch angle derivative gain[/s] 28.3 50
g_spado = 2*pi*6; %cut-off angular frequency[rad/s] 4

%% Yaw angle
K_yp = 100; %yaw angle position gain[/s^2]
K_yd = 20; %yaw angle derivative gain[/s]
g_yaw = 2*pi*100; %cut-off angular frequency[rad/s]

%% Fork angle
K_fp = 200; %fork angle position gain[/s^2]
K_fd = 50; %fork angle derivative gain[/s]
g_fork = 2*pi*5; %cut-off angular frequency[rad/s]
g_reac = 2*pi*1; %cut-off angular frequency[rad/s]

%% Lift height
K_mp = 200; %lift height position gain[/s^2]
K_md = 50; %lift height derivative gain[/s]
g_lift = 2*pi*4; %cut-off angular frequency[rad/s]

%% Distance
K_wp = 0.07; %wheel angle position gain 0.07
K_wi = 0; %wheel angle integral gain 0
K_wd = 0.09; %wheel angle derivative gain 0.11 0.09

%% RCC
Ac = 1;%1
omg = 5;%5
gamm = 1;%1
Kc = 168.05; % 195.61 267
Mc = Kc/omg^2; %0.04 3.23 0.137
Dc = gamm*2*sqrt(Kc*Mc); %0.625 32.6 1.549
g_ext = 2*pi*15; %cut-off angular frequency[rad/s]