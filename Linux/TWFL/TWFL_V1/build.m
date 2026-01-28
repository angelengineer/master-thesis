%% Build all
script_dir = fileparts(mfilename('fullpath'));
cd(script_dir)

clear all; clc; clear mex;
fprintf("Deleting old code...\n");

d = dir('c_generated_code*');

for k = 1:length(d)
    if d(k).isdir
        rmdir(d(k).name, 's');
    end
end


if isfolder("build")
    rmdir("build", "s");   
end


% get default simulink_opts
simulink_opts = get_acados_simulink_opts;

%% compiling ocp
fprintf("Compiling ...\n");

d = dir('ocp*.m');

for k = 1:length(d)
    fprintf('Running %s\n', d(k).name);
    run(d(k).name);
end


%% Sfunctions
fprintf("Building sfunctions...\n");

d = dir('c_generated_code*');
base_dir = pwd;

for k = 1:length(d)
    if d(k).isdir && ~ismember(d(k).name, {'.','..'})
        cd(fullfile(base_dir, d(k).name));

        make_sfun;      % ocp solver
        make_sfun_sim;  % simulator

        cd(base_dir);
    end
end
%%
dynamics;
robotParams;
