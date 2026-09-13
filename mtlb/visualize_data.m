%% Visualize prepared training and evaluation datasets
% This script reads the saved MAT datasets. Use visualize_raw_data.m for
% rosbag timing, interpolation, and raw-message diagnostics.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% SETUP

training_dataset_file = fullfile(mtlb_dir,'data','datasets_training.mat');
evaluation_dataset_file = ...
    fullfile(mtlb_dir,'data','datasets_evaluation.mat');

% Selected prepared run to inspect.
selected_split = "evaluation"; % "training" or "evaluation"
selected_run = 1;               % index inside the selected split

% Leave empty for the complete prepared run, or use [first last] samples.
selected_sample_range = [];

% Exchange X and Y only in the trajectory visualization. This does not
% modify the loaded dataset or any model input.
swap_map_xy = true;

%% LOAD PREPARED DATA

assert(isfile(training_dataset_file), ...
    'Training dataset not found: %s',training_dataset_file);
assert(isfile(evaluation_dataset_file), ...
    'Evaluation dataset not found: %s',evaluation_dataset_file);

training_source = load(training_dataset_file,'datasets','meta');
evaluation_source = load(evaluation_dataset_file,'datasets','meta');
assert(isfield(training_source,'datasets') && ...
    isfield(training_source,'meta'),'Invalid training dataset file.');
assert(isfield(evaluation_source,'datasets') && ...
    isfield(evaluation_source,'meta'),'Invalid evaluation dataset file.');
assert(abs(training_source.meta.Ts-evaluation_source.meta.Ts) < ...
    eps(max(training_source.meta.Ts,evaluation_source.meta.Ts)), ...
    'Training and evaluation datasets have different sample times.');

fprintf('Loaded %d training and %d evaluation runs at Ts = %.4f s.\n', ...
    numel(training_source.datasets),numel(evaluation_source.datasets), ...
    training_source.meta.Ts);

%% DETAILED RUN AND METADATA TABLE

assert(exist('training_source','var')==1 && ...
    exist('evaluation_source','var')==1, ...
    'Run LOAD PREPARED DATA before building the table.');

run_table = build_run_table( ...
    training_source.datasets,evaluation_source.datasets);
fprintf('\nPREPARED DATASET RUNS AND METADATA\n');
disp(run_table);

fprintf('\nRUN COUNTS BY SPLIT AND EVENT\n');
disp(groupcounts(run_table,{'Split','Event'}));
fprintf('\nDECLARED LAPS BY SPLIT AND TRACK LAYOUT\n');
[lap_groups,lap_split,lap_layout] = findgroups( ...
    run_table.Split,run_table.TrackLayout);
declared_laps = splitapply(@sum,run_table.DeclaredLaps,lap_groups);
disp(table(lap_split,lap_layout,declared_laps, ...
    'VariableNames',{'Split','TrackLayout','DeclaredLaps'}));

%% SELECT RUN

assert(exist('training_source','var')==1 && ...
    exist('evaluation_source','var')==1, ...
    'Run LOAD PREPARED DATA before SELECT RUN.');
assert(any(strcmpi(selected_split,["training","evaluation"])), ...
    'selected_split must be "training" or "evaluation".');

if strcmpi(selected_split,"training")
    selected_datasets = training_source.datasets; %#ok<UNRCH>
else
    selected_datasets = evaluation_source.datasets;
end
validateattributes(selected_run,{'numeric'}, ...
    {'scalar','integer','>=',1,'<=',numel(selected_datasets)}, ...
    mfilename,'selected_run');
selected = selected_datasets(selected_run);
data = selected.data;

n_samples = numel(data.time);
if isempty(selected_sample_range)
    selected_indices = 1:n_samples;
else
    validateattributes(selected_sample_range,{'numeric'}, ...
        {'vector','numel',2,'integer','increasing','>=',1,'<=',n_samples}, ...
        mfilename,'selected_sample_range');
    selected_indices = selected_sample_range(1):selected_sample_range(2);
end

fprintf(['Selected %s run %d: %s | %s | track ID %g | %g lap(s) | ' ...
    '%d/%d samples\n'],selected_split,selected_run,selected.event, ...
    selected.track_layout,selected.track_id,selected.laps, ...
    numel(selected_indices),n_samples);
fprintf('Source rosbag: %s\n',selected.path);

%% VISUALIZE SELECTED RUN

assert(exist('selected','var')==1 && exist('selected_indices','var')==1, ...
    'Run SELECT RUN before VISUALIZE SELECTED RUN.');

idx = selected_indices;
t = data.time(idx);
figure_name = sprintf('%s run %02d - %s - %s', ...
    selected_split,selected_run,selected.event,selected.track_layout);
data_figure = figure('Name',figure_name,'Color','w', ...
    'Position',[80 80 1550 900]);
layout = tiledlayout(4,3,'TileSpacing','compact','Padding','compact');
title(layout,sprintf('%s run %d | %s | %s | %g declared lap(s)', ...
    upperFirst(selected_split),selected_run,selected.event, ...
    selected.track_layout,selected.laps),'Interpreter','none');

if swap_map_xy
    map_x = data.y(idx);
    map_y = data.x(idx);
    map_x_label = 'Y [m]';
    map_y_label = 'X [m]';
else
    map_x = data.x(idx); %#ok<UNRCH>
    map_y = data.y(idx);
    map_x_label = 'X [m]';
    map_y_label = 'Y [m]';
end

map_axis = nexttile(layout,[4 1]); hold(map_axis,'on'); grid(map_axis,'on');
plot(map_axis,map_x,map_y,'k-','LineWidth',1.3);
scatter(map_axis,map_x(1),map_y(1),50,[0.1 0.65 0.2], ...
    'filled','DisplayName','Start');
scatter(map_axis,map_x(end),map_y(end),50,[0.85 0.15 0.1], ...
    'filled','DisplayName','End');
axis(map_axis,'equal');
xlabel(map_axis,map_x_label); ylabel(map_axis,map_y_label);
title(map_axis,'Prepared planar trajectory'); legend(map_axis,'Location','best');

time_axes = gobjects(8,1);
time_axes(1) = nexttile(layout); plot(time_axes(1),t,data.vx(idx),'LineWidth',1);
ylabel(time_axes(1),'v_x [m/s]'); title(time_axes(1),'Longitudinal velocity');

time_axes(2) = nexttile(layout); plot(time_axes(2),t,data.vy(idx),'LineWidth',1);
ylabel(time_axes(2),'v_y [m/s]'); title(time_axes(2),'Lateral velocity');

time_axes(3) = nexttile(layout); plot(time_axes(3),t,data.r(idx),'LineWidth',1);
ylabel(time_axes(3),'r [rad/s]'); title(time_axes(3),'Yaw rate');

time_axes(4) = nexttile(layout); plot(time_axes(4),t,data.psi(idx),'LineWidth',1);
ylabel(time_axes(4),'\psi [rad]'); title(time_axes(4),'Planar body heading');

time_axes(5) = nexttile(layout); hold(time_axes(5),'on');
plot(time_axes(5),t,data.delta(idx),'LineWidth',1.1,'DisplayName','Measured');
plot(time_axes(5),t,data.st(idx),'LineWidth',0.9,'DisplayName','Command');
ylabel(time_axes(5),'\delta [rad]'); title(time_axes(5),'Steering');
legend(time_axes(5),'Location','best');

time_axes(6) = nexttile(layout); hold(time_axes(6),'on');
plot(time_axes(6),t,data.ax(idx),'LineWidth',1,'DisplayName','a_x');
plot(time_axes(6),t,data.ay(idx),'LineWidth',1,'DisplayName','a_y');
ylabel(time_axes(6),'a [m/s^2]'); title(time_axes(6),'Acceleration');
legend(time_axes(6),'Location','best');

time_axes(7) = nexttile(layout); plot(time_axes(7),t,data.mz(idx),'LineWidth',1);
ylabel(time_axes(7),'M_z [Nm]'); title(time_axes(7),'Yaw moment');

time_axes(8) = nexttile(layout); hold(time_axes(8),'on');
plot(time_axes(8),t,data.Tfl(idx),'DisplayName','FL');
plot(time_axes(8),t,data.Tfr(idx),'DisplayName','FR');
plot(time_axes(8),t,data.Trl(idx),'DisplayName','RL');
plot(time_axes(8),t,data.Trr(idx),'DisplayName','RR');
ylabel(time_axes(8),'Torque [Nm]'); xlabel(time_axes(8),'Time [s]');
title(time_axes(8),'Wheel torques'); legend(time_axes(8),'Location','best');

for axis_idx = 1:numel(time_axes)
    grid(time_axes(axis_idx),'on');
end
linkaxes(time_axes,'x');
save_script_figures('visualize_data',data_figure);

%% 


%% LOCAL FUNCTIONS

function result = build_run_table(training,evaluation)
sources = {training(:),evaluation(:)};
split_names = ["training","evaluation"];
n = numel(training)+numel(evaluation);

split = strings(n,1); run = zeros(n,1); event = strings(n,1);
track_id = NaN(n,1); track_layout = strings(n,1); laps = NaN(n,1);
samples = zeros(n,1); duration = NaN(n,1); sample_time = NaN(n,1);
configured_ini = NaN(n,1); configured_fin = NaN(n,1);
vx_min = NaN(n,1); vx_max = NaN(n,1); vy_rms = NaN(n,1);
yaw_rate_rms = NaN(n,1); max_abs_steering = NaN(n,1);
nonzero_mz = zeros(n,1); path = strings(n,1);

row = 0;
for source_idx = 1:numel(sources)
    datasets = sources{source_idx};
    for run_idx = 1:numel(datasets)
        row = row+1;
        item = datasets(run_idx);
        d = item.data;
        split(row) = split_names(source_idx);
        run(row) = run_idx;
        event(row) = string(item.event);
        track_id(row) = double(item.track_id);
        track_layout(row) = string(item.track_layout);
        laps(row) = double(item.laps);
        samples(row) = numel(d.time);
        if samples(row)>1
            duration(row) = d.time(end)-d.time(1);
            sample_time(row) = median(diff(d.time));
        end
        if ~isempty(item.ini), configured_ini(row) = double(item.ini); end
        if ~isempty(item.fin), configured_fin(row) = double(item.fin); end
        vx_min(row) = min(d.vx,[],'omitnan');
        vx_max(row) = max(d.vx,[],'omitnan');
        vy_rms(row) = sqrt(mean(d.vy.^2,'omitnan'));
        yaw_rate_rms(row) = sqrt(mean(d.r.^2,'omitnan'));
        max_abs_steering(row) = max(abs(d.delta),[],'omitnan');
        nonzero_mz(row) = nnz(abs(d.mz)>1e-9);
        path(row) = string(item.path);
    end
end

result = table(split,run,event,track_id,track_layout,laps,samples, ...
    duration,sample_time,configured_ini,configured_fin,vx_min,vx_max, ...
    vy_rms,yaw_rate_rms,max_abs_steering,nonzero_mz,path, ...
    'VariableNames',{'Split','Run','Event','TrackID','TrackLayout', ...
    'DeclaredLaps','Samples','Duration_s','SampleTime_s','ConfiguredIni', ...
    'ConfiguredFin','MinVx_m_s','MaxVx_m_s','VyRMS_m_s', ...
    'YawRateRMS_rad_s','MaxAbsSteering_rad','NonzeroMzSamples', ...
    'SourcePath'});
end

function value = upperFirst(value)
value = char(string(value));
value(1) = upper(value(1));
end
