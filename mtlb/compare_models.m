cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear;
%% Setup
Ts = 0.02;
dataset_file = fullfile("data", "datasets_evaluation.mat");

% Models to compare. Keep each colour fixed across every figure so a model
% has the same visual identity in summary, residual, and propagation plots.
models = {
    @anfis_direct,          'ANFIS direct',          [0.000, 0.447, 0.741];
    @anfis_delta,           'ANFIS delta',           [0.850, 0.325, 0.098];
    @anfis_dot,             'ANFIS derivative',      [0.929, 0.694, 0.125];
    @anfis_residuals,       'ANFIS LTV residual',    [0.494, 0.184, 0.556];
    @eefig,                 'EEFIG offline',         [0.466, 0.674, 0.188];
    @nonlinear_bicycle,     'Nonlinear bicycle',     [0.301, 0.745, 0.933];
    @nonlinear_double_track,'Nonlinear double track',[0.635, 0.078, 0.184];
    @ltv,                   'LTV MPC',               [0.650, 0.650, 0.650];
};
model_colors = vertcat(models{:,3});

% One-step evaluation: evaluate next-state prediction.
% - Evaluate one-step prediction x_{k+1} from measured x_k and measured inputs u_k.
eval_cfg.suppress_model_prints = true;      % avoids ANFIS extrapolation spam during batch evaluation
eval_cfg.vx_edges = [0:2:30 Inf];           % bins for "where the model is bad"
eval_cfg.delta_abs_edges = [0:0.02:0.40 Inf]; % |delta| bins [rad]
eval_cfg.vy_edges = [-3:0.25:3 Inf];        % bins for residual analysis
eval_cfg.r_edges  = [-2:0.15:2 Inf];        % bins for residual analysis

% Simulate on one run (state propagation)
sim_cfg.run_idx = 1;
sim_cfg.idx_start = 400;
sim_cfg.horizon = 60;


%% Load prepared evaluation datasets
if ~isfile(dataset_file)
    error(['Evaluation dataset not found: %s\n' ...
           'Run prepare_datasets.m before comparing models.'], dataset_file);
end

load(dataset_file, 'datasets', 'meta');

if abs(meta.Ts - Ts) > eps(max(meta.Ts, Ts))
    error('Dataset sampling time (%g s) does not match model Ts (%g s).', ...
          meta.Ts, Ts);
end

runs = cell(numel(datasets), 1);

for i = 1:numel(datasets)
    bagPath = datasets(i).path;
    data = datasets(i).data;

    ini = datasets(i).ini;
    if isempty(ini)
        ini = 1;
    end
    fin = datasets(i).fin;
    if isempty(fin)
        fin = numel(data.vx);
    end
    fin = min(fin, numel(data.vx));

    idx = ini:fin;

    run.path = bagPath;
    run.event = datasets(i).event;
    run.laps = datasets(i).laps;
    run.track_id = datasets(i).track_id;
    run.track_layout = datasets(i).track_layout;
    run.t = data.time(idx);
    if ~isfield(data, 'psi')
        error(['Dataset %d has no measured heading. Regenerate the prepared ' ...
            'datasets with the current prepare_datasets.m.'], i);
    end
    run.psi = data.psi(idx);
    run.vx = data.vx(idx);
    run.vy = data.vy(idx);
    run.r = data.r(idx);
    run.delta = data.delta(idx);

    % Use recorded body yaw only. SLAM x/y are not used in the dynamics
    % comparison because their trajectory direction is unreliable and is
    % not a substitute for vehicle heading under sideslip.
    run.psi_local = unwrap(run.psi(:));
    run.psi_local = run.psi_local - run.psi_local(1);

    runs{i} = run;
    fprintf(['Rosbag %d prepared: %s | %s | %s | %g lap(s) | ' ...
             '%d points\n'], ...
            i, bagPath, run.event, run.track_layout, run.laps, numel(run.vx));
end


%% One-step evaluation (no propagation)
% Predict x_{k+1} from measured x_k, vx_k, and delta_k. Steering-command
% and steering-actuator dynamics are outside this model comparison.

res = struct();
res.models = models(:,2);
res.e_vy = cell(size(models,1),1);
res.e_psi = cell(size(models,1),1);
res.e_r  = cell(size(models,1),1);
res.vx = [];
res.delta = [];
res.vy = [];
res.r = [];
res.run_id = [];

for m = 1:size(models,1)
    modelFcn = models{m,1};
    name = models{m,2};
    fprintf('\nEvaluating model: %s\n', name);

    evy = [];
    epsi = [];
    er  = [];
    vx_k = [];
    delta_k = [];
    vy_meas_k = [];
    r_meas_k = [];
    run_id_k = [];

    for i = 1:numel(runs)
        run = runs{i};
        N = numel(run.t);
        if N < 2
            continue
        end

        vx = run.vx(:);
        delta = run.delta(:);

        % Build model state at k from measured signals
        Xk = zeros(6, N-1);
        Xk(1,:) = 0;
        Xk(2,:) = run.vy(1:end-1);
        Xk(3,:) = run.psi_local(1:end-1);
        Xk(4,:) = run.r(1:end-1);
        Xk(5,:) = run.delta(1:end-1);
        Xk(6,:) = 0;

        % Measured delta is held constant within each 20-ms transition.
        % Setting legacy actuator state delta_dot=0 and command=delta puts
        % that subsystem at equilibrium and removes it from vy/r prediction.
        Uk = [vx(1:end-1), delta(1:end-1)];

        vy_next = run.vy(2:end);
        psi_next = run.psi_local(2:end);
        r_next  = run.r(2:end);

        for k = 1:(N-1)
            if eval_cfg.suppress_model_prints
                [~, x_next] = evalc('modelFcn(Xk(:,k), Uk(k,:), Ts)');
            else
                x_next = modelFcn(Xk(:,k), Uk(k,:), Ts);
            end

            evy(end+1,1) = x_next(2) - vy_next(k);
            epsi(end+1,1) = wrapAngle(x_next(3) - psi_next(k));
            er(end+1,1)  = x_next(4) - r_next(k);
            vx_k(end+1,1) = vx(k);
            delta_k(end+1,1) = delta(k);
            vy_meas_k(end+1,1) = run.vy(k);
            r_meas_k(end+1,1) = run.r(k);
            run_id_k(end+1,1) = i;
        end
    end

    res.e_vy{m} = evy;
    res.e_psi{m} = epsi;
    res.e_r{m}  = er;
    if isempty(res.vx)
        res.vx = vx_k;
        res.delta = delta_k;
        res.vy = vy_meas_k;
        res.r = r_meas_k;
        res.run_id = run_id_k;
    end
end


%% Statistics
stats = struct();
stats.models = res.models;

stats.rmse_vy = zeros(size(models,1),1);
stats.rmse_psi = zeros(size(models,1),1);
stats.rmse_r  = zeros(size(models,1),1);

stats.mae_vy  = zeros(size(models,1),1);
stats.mae_psi = zeros(size(models,1),1);
stats.mae_r   = zeros(size(models,1),1);

stats.p95abs_vy = zeros(size(models,1),1);
stats.p95abs_psi = zeros(size(models,1),1);
stats.p95abs_r  = zeros(size(models,1),1);

for m = 1:size(models,1)
    evy = res.e_vy{m};
    epsi = res.e_psi{m};
    er  = res.e_r{m};

    stats.rmse_vy(m) = sqrt(mean(evy.^2,'omitnan'));
    stats.rmse_psi(m) = sqrt(mean(epsi.^2,'omitnan'));
    stats.rmse_r(m)  = sqrt(mean(er.^2,'omitnan'));

    stats.mae_vy(m)  = mean(abs(evy),'omitnan');
    stats.mae_psi(m) = mean(abs(epsi),'omitnan');
    stats.mae_r(m)   = mean(abs(er),'omitnan');

    stats.p95abs_vy(m) = prctile(abs(evy),95);
    stats.p95abs_psi(m) = prctile(abs(epsi),95);
    stats.p95abs_r(m)  = prctile(abs(er),95);
end

T_rmse = table(string(stats.models), stats.rmse_vy, stats.rmse_r, ...
    'VariableNames', {'model','rmse_vy','rmse_r'});
disp(T_rmse)

T_mae = table(string(stats.models), stats.mae_vy, stats.mae_r, ...
    'VariableNames', {'model','mae_vy','mae_r'});
disp(T_mae)

T_p95 = table(string(stats.models), stats.p95abs_vy, stats.p95abs_r, ...
    'VariableNames', {'model','p95abs_vy','p95abs_r'});
disp(T_p95)

% Residuals vs speed bins
vx = res.vx(:);
delta = res.delta(:);
vx_edges = eval_cfg.vx_edges(:)';
delta_abs_edges = eval_cfg.delta_abs_edges(:)';
vy_edges = eval_cfg.vy_edges(:)';
r_edges  = eval_cfg.r_edges(:)';

% Replace Inf edges with data-driven finite caps (needed for plotting X/YData)
if any(~isfinite(vx_edges))
    vx_cap = max([max(vx,[],'omitnan'), max(vx_edges(isfinite(vx_edges)))]) + 1e-6;
    vx_edges(~isfinite(vx_edges)) = vx_cap;
end
if any(~isfinite(delta_abs_edges))
    delta_cap = max([max(abs(delta),[],'omitnan'), max(delta_abs_edges(isfinite(delta_abs_edges)))]) + 1e-6;
    delta_abs_edges(~isfinite(delta_abs_edges)) = delta_cap;
end
if any(~isfinite(vy_edges))
    vy_cap = max([max(res.vy,[],'omitnan'), max(vy_edges(isfinite(vy_edges)))]) + 1e-6;
    vy_edges(~isfinite(vy_edges)) = vy_cap;
end
if any(~isfinite(r_edges))
    r_cap = max([max(res.r,[],'omitnan'), max(r_edges(isfinite(r_edges)))]) + 1e-6;
    r_edges(~isfinite(r_edges)) = r_cap;
end

vx_centers = 0.5*(vx_edges(1:end-1) + vx_edges(2:end));
delta_centers = 0.5*(delta_abs_edges(1:end-1) + delta_abs_edges(2:end));
vy_centers = 0.5*(vy_edges(1:end-1) + vy_edges(2:end));
r_centers  = 0.5*(r_edges(1:end-1) + r_edges(2:end));

stats.vx_edges = vx_edges;
stats.vx_centers = vx_centers;
stats.delta_abs_edges = delta_abs_edges;
stats.delta_abs_centers = delta_centers;
stats.vy_edges = vy_edges;
stats.vy_centers = vy_centers;
stats.r_edges = r_edges;
stats.r_centers = r_centers;

stats.rmse_vy_vs_vx = NaN(size(models,1), numel(vx_centers));
stats.rmse_r_vs_vx  = NaN(size(models,1), numel(vx_centers));

stats.rmse_vy_vs_vy = NaN(size(models,1), numel(vy_centers));
stats.rmse_r_vs_vy  = NaN(size(models,1), numel(vy_centers));

stats.rmse_vy_vs_r = NaN(size(models,1), numel(r_centers));
stats.rmse_r_vs_r  = NaN(size(models,1), numel(r_centers));

stats.rmse_vy_map = NaN(size(models,1), numel(delta_centers), numel(vx_centers));
stats.rmse_r_map  = NaN(size(models,1), numel(delta_centers), numel(vx_centers));

vx_bin = discretize(vx, vx_edges);
delta_bin = discretize(abs(delta), delta_abs_edges);
vy_bin = discretize(res.vy(:), vy_edges);
r_bin  = discretize(res.r(:), r_edges);

for m = 1:size(models,1)
    evy = res.e_vy{m};
    er  = res.e_r{m};

    for b = 1:numel(vx_centers)
        mask = (vx_bin == b);
        if any(mask)
            stats.rmse_vy_vs_vx(m,b) = sqrt(mean(evy(mask).^2,'omitnan'));
            stats.rmse_r_vs_vx(m,b)  = sqrt(mean(er(mask).^2,'omitnan'));
        end
    end

    for b = 1:numel(vy_centers)
        mask = (vy_bin == b);
        if any(mask)
            stats.rmse_vy_vs_vy(m,b) = sqrt(mean(evy(mask).^2,'omitnan'));
            stats.rmse_r_vs_vy(m,b)  = sqrt(mean(er(mask).^2,'omitnan'));
        end
    end

    for b = 1:numel(r_centers)
        mask = (r_bin == b);
        if any(mask)
            stats.rmse_vy_vs_r(m,b) = sqrt(mean(evy(mask).^2,'omitnan'));
            stats.rmse_r_vs_r(m,b)  = sqrt(mean(er(mask).^2,'omitnan'));
        end
    end

    for bb_vx = 1:numel(vx_centers)
        for bb_d = 1:numel(delta_centers)
            mask = (vx_bin == bb_vx) & (delta_bin == bb_d);
            if any(mask)
                stats.rmse_vy_map(m,bb_d,bb_vx) = sqrt(mean(evy(mask).^2,'omitnan'));
                stats.rmse_r_map(m,bb_d,bb_vx)  = sqrt(mean(er(mask).^2,'omitnan'));
            end
        end
    end
end


%% Visualization (statistics)

% Lateral-dynamics errors. Steering-angle errors are omitted because every
% model uses the same steering-actuator dynamics.
figure('Name','Model errors (one-step)','Position',[100 100 1200 650]);
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl,1);
plotErrorSummary(ax1, stats.models, stats.rmse_r, stats.mae_r, stats.p95abs_r, ...
    model_colors, 'r error [rad/s]', 'Yaw rate');

ax2 = nexttile(tl,2);
plotErrorSummary(ax2, stats.models, stats.rmse_vy, stats.mae_vy, stats.p95abs_vy, ...
    model_colors, 'v_y error [m/s]', 'Lateral velocity');


% Residuals vs signals (vx, vy, r)
figure('Name','Residuals vs signals','Position',[100 100 1400 800]);
tl = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl,1); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vx_centers, stats.rmse_vy_vs_vx(m,:), ...
        'Color', model_colors(m,:), 'LineWidth', 1.4, ...
        'DisplayName', stats.models{m});
end
xlabel('v_x [m/s]'); ylabel('RMSE e_{vy} [m/s]');
title('vy residual vs v_x');
legend('Location','best');

ax2 = nexttile(tl,2); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vy_centers, stats.rmse_vy_vs_vy(m,:), ...
        'Color', model_colors(m,:), 'LineWidth', 1.4, ...
        'DisplayName', stats.models{m});
end
xlabel('v_y [m/s]'); ylabel('RMSE e_{vy} [m/s]');
title('vy residual vs v_y');
legend('Location','best');

ax3 = nexttile(tl,3); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.r_centers, stats.rmse_vy_vs_r(m,:), ...
        'Color', model_colors(m,:), 'LineWidth', 1.4, ...
        'DisplayName', stats.models{m});
end
xlabel('r [rad/s]'); ylabel('RMSE e_{vy} [m/s]');
title('vy residual vs r');
legend('Location','best');

ax4 = nexttile(tl,4); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vx_centers, stats.rmse_r_vs_vx(m,:), ...
        'Color', model_colors(m,:), 'LineWidth', 1.4, ...
        'DisplayName', stats.models{m});
end
xlabel('v_x [m/s]'); ylabel('RMSE e_r [rad/s]');
title('r residual vs v_x');
legend('Location','best');

ax5 = nexttile(tl,5); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vy_centers, stats.rmse_r_vs_vy(m,:), ...
        'Color', model_colors(m,:), 'LineWidth', 1.4, ...
        'DisplayName', stats.models{m});
end
xlabel('v_y [m/s]'); ylabel('RMSE e_r [rad/s]');
title('r residual vs v_y');
legend('Location','best');

ax6 = nexttile(tl,6); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.r_centers, stats.rmse_r_vs_r(m,:), ...
        'Color', model_colors(m,:), 'LineWidth', 1.4, ...
        'DisplayName', stats.models{m});
end
xlabel('r [rad/s]'); ylabel('RMSE e_r [rad/s]');
title('r residual vs r');
legend('Location','best');


%% Propagation on one dataset window (simulateModel)
run = runs{sim_cfg.run_idx};
idx_start = sim_cfg.idx_start;
horizon = sim_cfg.horizon;
idx_end = min(idx_start + horizon - 1, numel(run.vx));

meas.t = run.t(idx_start:idx_end);
in.t = run.t(idx_start:idx_end);

in.vx = run.vx(idx_start:idx_end);
in.delta = run.delta(idx_start:idx_end);

meas.vx = run.vx(idx_start:idx_end);
meas.vy = run.vy(idx_start:idx_end);
meas.r = run.r(idx_start:idx_end);
meas.delta = run.delta(idx_start:idx_end);

init_state.y = 0;
init_state.vy = meas.vy(1);
init_state.psi = 0;
init_state.r = meas.r(1);
init_state.delta = meas.delta(1);

% Run each model on the selected window (propagation)
sim_results = cell(size(models,1),1);
for k = 1:size(models,1)
    modelFcn = models{k,1};
    name     = models{k,2};
    fprintf('Simulating model (propagation): %s\n', name);
    sim_results{k} = simulateModel(modelFcn, in, init_state);
end

% Recorded planar body yaw relative to the start of this window.
psi_local = unwrap(run.psi(idx_start:idx_end));
psi_local = psi_local - psi_local(1);

meas_local.t   = meas.t - meas.t(1);
meas_local.psi = psi_local;
meas_local.vy  = meas.vy(:);
meas_local.r   = meas.r(:);

figure('Name','Propagation (Local Frame)','Position',[100 100 1200 800]);
tl = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

axDelta = nexttile(tl,1); hold on; grid on; ylabel('\delta [rad]'); title('Measured wheel-angle input');
axVy    = nexttile(tl,2); hold on; grid on; ylabel('v_y [m/s]'); title('Lateral velocity');
axR     = nexttile(tl,3); hold on; grid on; ylabel('r [rad/s]'); title('Yaw rate');
axPsi   = nexttile(tl,4); hold on; grid on; ylabel('\psi_{local} [rad]'); title('Measured body heading'); xlabel('t [s]');

plot(axDelta, meas_local.t, meas.delta(:),  'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axVy,  meas_local.t, meas_local.vy,  'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axR,   meas_local.t, meas_local.r,   'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axPsi, meas_local.t, meas_local.psi, 'w', 'LineWidth', 2, 'DisplayName','Measured');

for k = 1:size(models,1)
    name = models{k,2};
    states = sim_results{k};

    vy_sim  = arrayfun(@(s) s.vy,  states(:));
    r_sim   = arrayfun(@(s) s.r,   states(:));
    psi_sim = arrayfun(@(s) s.psi, states(:));

    t_sim = meas_local.t;
    color = model_colors(k,:);
    plot(axVy, t_sim, vy_sim, 'Color', color, ...
        'LineWidth', 1.4, 'DisplayName', name);
    plot(axR, t_sim, r_sim, 'Color', color, ...
        'LineWidth', 1.4, 'DisplayName', name);
    plot(axPsi, t_sim, psi_sim, 'Color', color, ...
        'LineWidth', 1.4, 'DisplayName', name);
end

legend(axVy,'Location','best');
legend(axR,'Location','best');
legend(axPsi,'Location','best');

linkaxes([axDelta axVy axR axPsi],'x');
%% 




%% Simulate model function
function states = simulateModel(modelFcn, inputs, init_state)
    dt = mean(diff(inputs.t));
    N  = length(inputs.t);
    
    % Convert init_state struct vector (+delta +delta_dot)
    x = [init_state.y, init_state.vy, init_state.psi, init_state.r, init_state.delta, 0]';
    
    states(N,1) = init_state;
    
    for k = 1:N
        % Treat measured steering position as a zero-order-held exogenous
        % input. The legacy actuator states are placed at equilibrium on
        % every step, so only the recursively propagated vehicle states
        % contribute to the comparison.
        x(5) = inputs.delta(k);
        x(6) = 0;
        u = [inputs.vx(k), inputs.delta(k)];
    
        % Log state
        states(k).y         = x(1);
        states(k).vy        = x(2);
        states(k).psi       = x(3);
        states(k).r         = x(4);
        states(k).delta     = x(5);
        states(k).delta_dot = x(6);
    
        x = modelFcn(x, u, dt);           % propagate
    end
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end

function plotErrorSummary(ax, modelNames, rmse, mae, p95abs, ...
    modelColors, ylab, ttl)
hold(ax, 'on');
grid(ax, 'on');

modelNames = string(modelNames);
x = 1:numel(modelNames);
rmse_bars = bar(ax, x, rmse, 'FaceColor', 'flat', 'FaceAlpha', 0.9, ...
    'DisplayName', 'RMSE');
rmse_bars.CData = modelColors;

for model_idx = 1:numel(modelNames)
    plot(ax, x(model_idx), mae(model_idx), 'o', ...
        'Color', modelColors(model_idx,:), ...
        'MarkerFaceColor', modelColors(model_idx,:), ...
        'MarkerEdgeColor', 'w', 'LineWidth', 1.2, ...
        'MarkerSize', 7, 'HandleVisibility', 'off');
    plot(ax, x(model_idx), p95abs(model_idx), 'x', ...
        'Color', modelColors(model_idx,:), 'LineWidth', 2.0, ...
        'MarkerSize', 9, 'HandleVisibility', 'off');
end

% Metric legend: model identity is encoded by bar/marker colour and named
% on the x-axis; marker shape distinguishes MAE from P95.
mae_key = plot(ax, nan, nan, 'o', 'Color', [0.8 0.8 0.8], ...
    'MarkerFaceColor', [0.8 0.8 0.8], 'MarkerEdgeColor', 'w', ...
    'LineWidth', 1.2, 'MarkerSize', 7, 'DisplayName', 'MAE');
p95_key = plot(ax, nan, nan, 'x', 'Color', [0.8 0.8 0.8], ...
    'LineWidth', 2.0, 'MarkerSize', 9, 'DisplayName', 'P95(|e|)');

xticks(ax, x);
xticklabels(ax, modelNames);
xtickangle(ax, 0);
ylabel(ax, ylab);
title(ax, ttl);
legend(ax, [rmse_bars, mae_key, p95_key], ...
    {'RMSE','MAE','P95(|e|)'}, 'Location', 'best');
end
