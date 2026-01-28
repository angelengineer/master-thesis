
clear all; clc; clear mex;
fprintf("Building sfunctions...");
if isfolder("c_generated_code")
    rmdir("c_generated_code", "s");   
end
if isfolder("build")
    rmdir("build", "s");   
end


simulink_opts = get_acados_simulink_opts;
diff_robot_ocp;


%% Compile Sfunctions
cd c_generated_code
%%
make_sfun; % ocp solver
make_sfun_sim; % integrator
cd ..

robotParams;