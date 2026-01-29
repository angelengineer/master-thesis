%% Build all
this_file = matlab.desktop.editor.getActiveFilename;
script_dir = fileparts(this_file);
cd(script_dir);


clear all; clc; clear mex; rehash;

% fprintf("Deleting old code...\n");
% d = dir('c_generated_code*');
% 
% for k = 1:length(d)
%     if d(k).isdir
%         folder = d(k).name;
%         items = dir(fullfile(folder, '*'));  % lista todo dentro de la carpeta
%         for i = 1:length(items)
%             name = items(i).name;
%             if strcmp(name, '.') || strcmp(name, '..')
%                 continue
%             end
%             fullpath = fullfile(folder, name);
%             if items(i).isdir
%                 % elimina subcarpeta y su contenido
%                 rmdir(fullpath, 's');
%             else
%                 % elimina archivo
%                 delete(fullpath);
%             end
%         end
%     end
% end

if isfolder("build")
    rmdir("build", "s");   
end
fprintf("Old code deleted.\n");

%% Preparing folders

% pattern = 'model_*';
% 
% items = dir(pattern);
% 
% if isempty(items)
%     fprintf('No items match "%s".\n', pattern);
%     return
% end
% 
% for k = 1:numel(items)
%     name = items(k).name;
%     dirName = ['c_generated_code_' name];
%     if exist(dirName, 'dir')
%         fprintf('Directory exists: %s\n', dirName);
%     else
%         mkdirStatus = mkdir(dirName);
%         if mkdirStatus
%             fprintf('Created directory: %s\n', dirName);
%         else
%             warning('Failed to create directory: %s', dirName);
%         end
%     end
% end


%% get default simulink_opts
%acados_env_variables_linux
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
addpath(genpath(pwd)); 
dynamics;
robotParams;
