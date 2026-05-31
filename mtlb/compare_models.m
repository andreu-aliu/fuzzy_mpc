cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all;
%% Setup
Ts = 0.02;
cache_name = "datasets_training";

% Load evaluation files
data_paths = {"/home/andreu/SIMULATIONS/results_3/run_2/rosbag",  [], [];
              "/home/andreu/SIMULATIONS/results_3/run_3/rosbag",  [], [];
              "/home/andreu/SIMULATIONS/results_3/run_4/rosbag",  [], [];
              "/home/andreu/SIMULATIONS/results_3/run_5/rosbag",  [], [];
              "/home/andreu/SIMULATIONS/results_3/run_6/rosbag",  [], [];
              "/home/andreu/SIMULATIONS/results_3/run_9/rosbag",  [], [];
              "/home/andreu/SIMULATIONS/results_3/run_10/rosbag", [], [];
              "/home/andreu/SIMULATIONS/results_3/run_11/rosbag", [], [];
              "/home/andreu/SIMULATIONS/results_3/run_12/rosbag", [], [];
              "/home/andreu/SIMULATIONS/results_3/run_13/rosbag", [], [];
              };

% Models to compare
models = {
    @anfis_delta, 'ANFIS delta';
    @anfis_dot, 'ANFIS dot';
    @ltv, 'LTV MPC';
    @ltv_tv, 'LTV MPC with TV';
};

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
sim_cfg.horizon = 500;


%% Load datasets (only once)
load_ros2bag_datasets(data_paths, Ts, cache_name);

%% Prepare datasets
load(cache_name + ".mat", 'datasets', 'meta');

runs = cell(size(data_paths,1),1);

for i = 1:size(data_paths,1)
    bagPath = data_paths{i,1};
    data = datasets(i).data;

    ini = data_paths{i,2};
    if isempty(ini)
        ini = 1;
    end
    fin = data_paths{i,3};
    if isempty(fin)
        fin = numel(data.vx);
    end
    fin = min(fin, numel(data.vx));

    idx = ini:fin;

    run.path = bagPath;
    run.t = data.time(idx);
    run.x = data.x(idx);
    run.y = data.y(idx);
    run.vx = data.vx(idx);
    run.vy = data.vy(idx);
    run.r = data.r(idx);
    run.delta = data.delta(idx);
    run.st = data.st(idx);
    run.mz = data.mz(idx);

    % Local frame (y,psi) for optional propagation plots / completeness
    [run.y_local, run.psi_local] = localFrameFromXY(run.x, run.y, run.r, run.t);

    % Delta derivative (only used to build state for model signature)
    run.delta_dot = gradient(run.delta, Ts);

    runs{i} = run;
    fprintf('Rosbag %d prepared: %s (%d points)\n', i, bagPath, numel(run.vx));
end


%% One-step evaluation (no propagation)
% Predict x_{k+1} from measured x_k and measured inputs u_k.

res = struct();
res.models = models(:,2);
res.e_y = cell(size(models,1),1);
res.e_vy = cell(size(models,1),1);
res.e_psi = cell(size(models,1),1);
res.e_r  = cell(size(models,1),1);
res.e_delta = cell(size(models,1),1);
res.e_delta_dot = cell(size(models,1),1);
res.vx = [];
res.delta = [];
res.vy = [];
res.r = [];
res.run_id = [];

for m = 1:size(models,1)
    modelFcn = models{m,1};
    name = models{m,2};
    fprintf('\nEvaluating model: %s\n', name);

    ey  = [];
    evy = [];
    epsi = [];
    er  = [];
    edelta = [];
    edelta_dot = [];
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
        Xk(1,:) = run.y_local(1:end-1);
        Xk(2,:) = run.vy(1:end-1);
        Xk(3,:) = run.psi_local(1:end-1);
        Xk(4,:) = run.r(1:end-1);
        Xk(5,:) = run.delta(1:end-1);
        Xk(6,:) = run.delta_dot(1:end-1);

        % Measured inputs
        Uk = [vx(1:end-1), run.st(1:end-1), run.mz(1:end-1)];

        y_next = run.y_local(2:end);
        vy_next = run.vy(2:end);
        psi_next = run.psi_local(2:end);
        r_next  = run.r(2:end);
        delta_next = run.delta(2:end);
        delta_dot_next = run.delta_dot(2:end);

        for k = 1:(N-1)
            if eval_cfg.suppress_model_prints
                [~, x_next] = evalc('modelFcn(Xk(:,k), Uk(k,:), Ts)');
            else
                x_next = modelFcn(Xk(:,k), Uk(k,:), Ts);
            end

            ey(end+1,1)  = x_next(1) - y_next(k);
            evy(end+1,1) = x_next(2) - vy_next(k);
            epsi(end+1,1) = wrapAngle(x_next(3) - psi_next(k));
            er(end+1,1)  = x_next(4) - r_next(k);
            edelta(end+1,1) = x_next(5) - delta_next(k);
            edelta_dot(end+1,1) = x_next(6) - delta_dot_next(k);
            vx_k(end+1,1) = vx(k);
            delta_k(end+1,1) = delta(k);
            vy_meas_k(end+1,1) = run.vy(k);
            r_meas_k(end+1,1) = run.r(k);
            run_id_k(end+1,1) = i;
        end
    end

    res.e_y{m} = ey;
    res.e_vy{m} = evy;
    res.e_psi{m} = epsi;
    res.e_r{m}  = er;
    res.e_delta{m} = edelta;
    res.e_delta_dot{m} = edelta_dot;
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

stats.rmse_y = zeros(size(models,1),1);
stats.rmse_vy = zeros(size(models,1),1);
stats.rmse_psi = zeros(size(models,1),1);
stats.rmse_r  = zeros(size(models,1),1);
stats.rmse_delta = zeros(size(models,1),1);
stats.rmse_delta_dot = zeros(size(models,1),1);

stats.mae_y = zeros(size(models,1),1);
stats.mae_vy  = zeros(size(models,1),1);
stats.mae_psi = zeros(size(models,1),1);
stats.mae_r   = zeros(size(models,1),1);
stats.mae_delta = zeros(size(models,1),1);
stats.mae_delta_dot = zeros(size(models,1),1);

stats.p95abs_y = zeros(size(models,1),1);
stats.p95abs_vy = zeros(size(models,1),1);
stats.p95abs_psi = zeros(size(models,1),1);
stats.p95abs_r  = zeros(size(models,1),1);
stats.p95abs_delta = zeros(size(models,1),1);
stats.p95abs_delta_dot = zeros(size(models,1),1);

for m = 1:size(models,1)
    ey  = res.e_y{m};
    evy = res.e_vy{m};
    epsi = res.e_psi{m};
    er  = res.e_r{m};
    edelta = res.e_delta{m};
    edelta_dot = res.e_delta_dot{m};

    stats.rmse_y(m) = sqrt(mean(ey.^2,'omitnan'));
    stats.rmse_vy(m) = sqrt(mean(evy.^2,'omitnan'));
    stats.rmse_psi(m) = sqrt(mean(epsi.^2,'omitnan'));
    stats.rmse_r(m)  = sqrt(mean(er.^2,'omitnan'));
    stats.rmse_delta(m) = sqrt(mean(edelta.^2,'omitnan'));
    stats.rmse_delta_dot(m) = sqrt(mean(edelta_dot.^2,'omitnan'));

    stats.mae_y(m)  = mean(abs(ey),'omitnan');
    stats.mae_vy(m)  = mean(abs(evy),'omitnan');
    stats.mae_psi(m) = mean(abs(epsi),'omitnan');
    stats.mae_r(m)   = mean(abs(er),'omitnan');
    stats.mae_delta(m) = mean(abs(edelta),'omitnan');
    stats.mae_delta_dot(m) = mean(abs(edelta_dot),'omitnan');

    stats.p95abs_y(m) = prctile(abs(ey),95);
    stats.p95abs_vy(m) = prctile(abs(evy),95);
    stats.p95abs_psi(m) = prctile(abs(epsi),95);
    stats.p95abs_r(m)  = prctile(abs(er),95);
    stats.p95abs_delta(m) = prctile(abs(edelta),95);
    stats.p95abs_delta_dot(m) = prctile(abs(edelta_dot),95);
end

T_rmse = table(string(stats.models), stats.rmse_vy, stats.rmse_r, stats.rmse_delta, ...
    'VariableNames', {'model','rmse_vy','rmse_r','rmse_delta'});
disp(T_rmse)

T_mae = table(string(stats.models), stats.mae_vy, stats.mae_r, stats.mae_delta, ...
    'VariableNames', {'model','mae_vy','mae_r','mae_delta'});
disp(T_mae)

T_p95 = table(string(stats.models), stats.p95abs_vy, stats.p95abs_r, stats.p95abs_delta, ...
    'VariableNames', {'model','p95abs_vy','p95abs_r','p95abs_delta'});
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
stats.rmse_delta_vs_vx  = NaN(size(models,1), numel(vx_centers));

stats.rmse_vy_vs_vy = NaN(size(models,1), numel(vy_centers));
stats.rmse_r_vs_vy  = NaN(size(models,1), numel(vy_centers));

stats.rmse_vy_vs_r = NaN(size(models,1), numel(r_centers));
stats.rmse_r_vs_r  = NaN(size(models,1), numel(r_centers));

stats.rmse_vy_map = NaN(size(models,1), numel(delta_centers), numel(vx_centers));
stats.rmse_r_map  = NaN(size(models,1), numel(delta_centers), numel(vx_centers));
stats.rmse_delta_map  = NaN(size(models,1), numel(delta_centers), numel(vx_centers));

vx_bin = discretize(vx, vx_edges);
delta_bin = discretize(abs(delta), delta_abs_edges);
vy_bin = discretize(res.vy(:), vy_edges);
r_bin  = discretize(res.r(:), r_edges);

for m = 1:size(models,1)
    evy = res.e_vy{m};
    er  = res.e_r{m};
    edelta = res.e_delta{m};

    for b = 1:numel(vx_centers)
        mask = (vx_bin == b);
        if any(mask)
            stats.rmse_vy_vs_vx(m,b) = sqrt(mean(evy(mask).^2,'omitnan'));
            stats.rmse_r_vs_vx(m,b)  = sqrt(mean(er(mask).^2,'omitnan'));
            stats.rmse_delta_vs_vx(m,b) = sqrt(mean(edelta(mask).^2,'omitnan'));
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
                stats.rmse_delta_map(m,bb_d,bb_vx)  = sqrt(mean(edelta(mask).^2,'omitnan'));
            end
        end
    end
end


%% Visualization (statistics)

% Errors
figure('Name','Model errors (one-step)','Position',[100 100 1200 800]);
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl,1);
plotErrorSummary(ax1, stats.models, stats.rmse_delta, stats.mae_delta, stats.p95abs_delta, ...
    '\delta error [rad]', 'Delta');

ax2 = nexttile(tl,2);
plotErrorSummary(ax2, stats.models, stats.rmse_r, stats.mae_r, stats.p95abs_r, ...
    'r error [rad/s]', 'Yaw rate');

ax3 = nexttile(tl,3);
plotErrorSummary(ax3, stats.models, stats.rmse_vy, stats.mae_vy, stats.p95abs_vy, ...
    'v_y error [m/s]', 'Lateral velocity');


% Residuals vs signals (vx, vy, r)
figure('Name','Residuals vs signals','Position',[100 100 1400 800]);
tl = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl,1); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vx_centers, stats.rmse_vy_vs_vx(m,:), 'LineWidth', 1.4, 'DisplayName', stats.models{m});
end
xlabel('v_x [m/s]'); ylabel('RMSE e_{vy} [m/s]');
title('vy residual vs v_x');
legend('Location','best');

ax2 = nexttile(tl,2); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vy_centers, stats.rmse_vy_vs_vy(m,:), 'LineWidth', 1.4, 'DisplayName', stats.models{m});
end
xlabel('v_y [m/s]'); ylabel('RMSE e_{vy} [m/s]');
title('vy residual vs v_y');
legend('Location','best');

ax3 = nexttile(tl,3); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.r_centers, stats.rmse_vy_vs_r(m,:), 'LineWidth', 1.4, 'DisplayName', stats.models{m});
end
xlabel('r [rad/s]'); ylabel('RMSE e_{vy} [m/s]');
title('vy residual vs r');
legend('Location','best');

ax4 = nexttile(tl,4); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vx_centers, stats.rmse_r_vs_vx(m,:), 'LineWidth', 1.4, 'DisplayName', stats.models{m});
end
xlabel('v_x [m/s]'); ylabel('RMSE e_r [rad/s]');
title('r residual vs v_x');
legend('Location','best');

ax5 = nexttile(tl,5); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.vy_centers, stats.rmse_r_vs_vy(m,:), 'LineWidth', 1.4, 'DisplayName', stats.models{m});
end
xlabel('v_y [m/s]'); ylabel('RMSE e_r [rad/s]');
title('r residual vs v_y');
legend('Location','best');

ax6 = nexttile(tl,6); hold on; grid on;
for m = 1:size(models,1)
    plot(stats.r_centers, stats.rmse_r_vs_r(m,:), 'LineWidth', 1.4, 'DisplayName', stats.models{m});
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

in.st = run.st(idx_start:idx_end);
in.mz = run.mz(idx_start:idx_end);
in.vx = run.vx(idx_start:idx_end);

meas.x = run.x(idx_start:idx_end);
meas.y = run.y(idx_start:idx_end);
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

% Local frame for the selected window
[y_local, psi_local] = localFrameFromXY(meas.x, meas.y, meas.r, meas.t);

meas_local.t   = meas.t - meas.t(1);
meas_local.y   = y_local;
meas_local.psi = psi_local;
meas_local.vy  = meas.vy(:);
meas_local.r   = meas.r(:);

figure('Name','Propagation (Local Frame)','Position',[100 100 1200 800]);
tl = tiledlayout(5,1,'TileSpacing','compact','Padding','compact');

axY     = nexttile(tl,1); hold on; grid on; ylabel('y_{local} [m]'); title('Lateral position');
axDelta = nexttile(tl,2); hold on; grid on; ylabel('\delta [rad]'); title('Wheel angle');
axVy    = nexttile(tl,3); hold on; grid on; ylabel('v_y [m/s]'); title('Lateral velocity');
axR     = nexttile(tl,4); hold on; grid on; ylabel('r [rad/s]'); title('Yaw rate');
axPsi   = nexttile(tl,5); hold on; grid on; ylabel('\psi_{local} [rad]'); title('Heading'); xlabel('t [s]');

plot(axY,     meas_local.t, meas_local.y,   'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axDelta, meas_local.t, meas.delta(:),  'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axVy,  meas_local.t, meas_local.vy,  'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axR,   meas_local.t, meas_local.r,   'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axPsi, meas_local.t, meas_local.psi, 'w', 'LineWidth', 2, 'DisplayName','Measured');

for k = 1:size(models,1)
    name = models{k,2};
    states = sim_results{k};

    y_sim   = arrayfun(@(s) s.y,   states(:));
    delta_sim = arrayfun(@(s) s.delta, states(:));
    vy_sim  = arrayfun(@(s) s.vy,  states(:));
    r_sim   = arrayfun(@(s) s.r,   states(:));
    psi_sim = arrayfun(@(s) s.psi, states(:));

    t_sim = meas_local.t;
    plot(axY,     t_sim, y_sim,     'LineWidth', 1.4, 'DisplayName', name);
    plot(axDelta, t_sim, delta_sim, 'LineWidth', 1.4, 'DisplayName', name);
    plot(axVy,  t_sim, vy_sim,  'LineWidth', 1.4, 'DisplayName', name);
    plot(axR,   t_sim, r_sim,   'LineWidth', 1.4, 'DisplayName', name);
    plot(axPsi, t_sim, psi_sim, 'LineWidth', 1.4, 'DisplayName', name);
end

legend(axY,'Location','best');
legend(axDelta,'Location','best');
legend(axVy,'Location','best');
legend(axR,'Location','best');
legend(axPsi,'Location','best');

linkaxes([axY axDelta axVy axR axPsi],'x');
%% 




%% Simulate model function
function states = simulateModel(modelFcn, inputs, init_state)
    dt = mean(diff(inputs.t));
    N  = length(inputs.t);
    
    % Convert init_state struct vector (+delta +delta_dot)
    x = [init_state.y, init_state.vy, init_state.psi, init_state.r, init_state.delta, 0]';
    
    states(N,1) = init_state;
    
    for k = 1:N
        % Get input
        u = [inputs.vx(k), inputs.st(k), inputs.mz(k)];
    
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

function [y_local, psi_local] = localFrameFromXY(xg, yg, r, t)
% Heading from trajectory (robust for sim logs). Fallback to yaw-rate if needed.
xg = xg(:); yg = yg(:);
if numel(xg) < 2
    y_local = zeros(size(xg));
    psi_local = zeros(size(xg));
    return
end
dx = gradient(xg);
dy = gradient(yg);

psi_traj = unwrap(atan2(dy, dx));

if all(abs(dx) < 1e-6) && all(abs(dy) < 1e-6)
    dt_vec = [diff(t(:)); mean(diff(t(:)))];
    psi_traj = unwrap(cumsum(r(:) .* dt_vec));
end

psi0 = psi_traj(1);
x0 = xg(1); y0 = yg(1);
dX = xg - x0;
dY = yg - y0;

y_local = -dX*sin(psi0) + dY*cos(psi0);
psi_local = unwrap(psi_traj - psi0);
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end

function plotErrorSummary(ax, modelNames, rmse, mae, p95abs, ylab, ttl)
axes(ax); %#ok<LAXES>
hold on; grid on;

modelNames = string(modelNames);
x = categorical(modelNames);
bar(x, rmse, 'FaceAlpha', 0.9);
plot(x, mae, 'o', 'Color', 'w', 'MarkerFaceColor','w', 'MarkerEdgeColor','w', ...
    'LineWidth', 1.2, 'MarkerSize', 7, 'DisplayName','MAE');
plot(x, p95abs, 'x', 'Color', [1 0.9 0], 'LineWidth', 1.8, 'MarkerSize', 8, ...
    'DisplayName','P95(|e|)');

ylabel(ylab);
title(ttl);
legend('RMSE','MAE','P95(|e|)','Location','best');
end
