clear;
mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));
%% Setup
dataset_file = fullfile(mtlb_dir,'data','datasets_evaluation.mat');
evaluation_run = 1;

% Simulation window (from data)
idx_start = 3000;
n  = 500;
Np = 60;
nx = 6;
nu = 1;
dt = 0.02;

%% LOAD DATA
source = load(dataset_file,'datasets','meta');
assert(evaluation_run>=1 && evaluation_run<=numel(source.datasets), ...
    'evaluation_run must be between 1 and %d.',numel(source.datasets));
assert(abs(source.meta.Ts-dt)<eps(max(source.meta.Ts,dt)), ...
    'Dataset sample time %.6g s does not match MPC Ts %.6g s.', ...
    source.meta.Ts,dt);
selected_run = source.datasets(evaluation_run);
data = selected_run.data;
fprintf('Evaluation run %d: %s | %s | %d lap(s) | %d samples\n', ...
    evaluation_run,selected_run.event,selected_run.track_layout, ...
    selected_run.laps,numel(data.time));

%% TAKE A WINDOW OF DATA
assert(idx_start>=1 && idx_start<=numel(data.vx), ...
    'idx_start must be between 1 and %d for evaluation run %d.', ...
    numel(data.vx),evaluation_run);
idx_end = min(idx_start+n-1,numel(data.vx));

meas.t = data.time(idx_start:idx_end);
in.t = data.time(idx_start:idx_end);

in.st = data.st(idx_start:idx_end);
in.vx = data.vx(idx_start:idx_end);

meas.x = data.x(idx_start:idx_end);
meas.y = data.y(idx_start:idx_end);
meas.vx = data.vx(idx_start:idx_end);
meas.vy = data.vy(idx_start:idx_end);
meas.r = data.r(idx_start:idx_end);
meas.delta = data.delta(idx_start:idx_end);

meas.course = compute_path_heading(meas.x,meas.y);
if isfield(data,'psi')
    meas.psi = unwrap(data.psi(idx_start:idx_end));
else
    % Course = body heading + sideslip. The legacy simulation MAT file has
    % no body-heading signal, so reconstruct it consistently with vx/vy.
    meas.psi = unwrap(meas.course-atan2(meas.vy,meas.vx));
end
n = numel(meas.t);

%% CHECK DATA LOADED
figure('Name','Data check','Position',[100 100 1200 600]);

% Main layout: 1 row, 2 columns
mainLayout = tiledlayout(1,2);

% --- Left: trajectory map ---
nexttile(mainLayout,1); hold on; grid on; axis equal;
plot(data.x, data.y, 'k');                    % full run
plot(meas.x, meas.y, 'b', 'LineWidth',1.5);                   % selected window
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory map');
legend('Full run','Selected window');

% --- Right: nested stacked time series ---
rightLayout = tiledlayout(mainLayout,4,1,'TileSpacing','compact','Padding','compact');
rightLayout.Layout.Tile = 2; 

% Longitudinal velocity
ax1 = nexttile(rightLayout); hold on;
plot(data.time, data.vx, 'k');
plot(meas.t, in.vx, 'b','LineWidth',1.2);
ylabel('V_x [m/s]');
title('Longitudinal velocity');
grid on

% Lateral velocity
ax2 = nexttile(rightLayout); hold on;
plot(data.time, data.vy, 'k');
plot(meas.t, meas.vy, 'b','LineWidth',1.2);
ylabel('V_y [m/s]');
title('Lateral velocity');
grid on

% Yaw rate
ax3 = nexttile(rightLayout); hold on;
plot(data.time, data.r, 'k');
plot(meas.t, meas.r, 'b','LineWidth',1.2);
ylabel('r [rad/s]');
title('Yaw rate');
grid on

% Measured steering position and recorded steering command
ax4 = nexttile(rightLayout); hold on;
plot(data.time,data.delta,'k');
plot(meas.t,meas.delta,'b','LineWidth',1.2);
plot(meas.t,in.st,'Color',[0.850 0.325 0.098],'LineWidth',1.0);
ylabel('\delta [rad]');
title('Steering');
xlabel('Time [s]');
legend('Full measured position','Window measured position', ...
    'Window recorded command');

% Link time axes
linkaxes([ax1 ax2 ax3 ax4],'x');
grid on

%% BUILD TRAJECTORY 

ds = 0.025;  % [m]

% Compute cumulative arc-length
dx = diff(meas.x);
dy = diff(meas.y);
s  = [0; cumsum(sqrt(dx.^2 + dy.^2))];
[s,unique_idx] = unique(s,'stable');
assert(numel(s)>=2,'The selected trajectory has no usable arc length.');
path_x = meas.x(unique_idx);
path_y = meas.y(unique_idx);
path_vx = meas.vx(unique_idx);

% Uniform arc-length grid
s_uniform = (0:ds:s(end)).';

% Interpolate
x_u = interp1(s,path_x,s_uniform,'pchip');
y_u = interp1(s,path_y,s_uniform,'pchip');
vx_u = interp1(s,path_vx,s_uniform,'linear');

% Compute heading
dx_u = gradient(x_u, ds);
dy_u = gradient(y_u, ds);
psi_u = unwrap(atan2(dy_u, dx_u));
kappa_u = gradient(psi_u,ds);

traj.s   = s_uniform;
traj.x   = x_u;
traj.y   = y_u;
traj.psi = psi_u;
traj.vx  = vx_u;
traj.kappa = kappa_u;


%% MPC PARAMETERS & OPTIONS

params.n_horizon = Np;
params.Ts = dt;
params.model = "eefig"; % ltv anfis_direct anfis_delta anfis_derivative anfis_ltv_residual eefig
params.eefig_adaptive = true;

% Scales for normalization
params.scale_y   = 0.1;   % m
params.scale_vy  = 0.1;   % m/s
params.scale_psi = 0.05;  % rad
params.scale_r   = 0.05;   % rad/s
params.scale_st  = 0.2;   % rad
params.scale_dst = 0.001;   % rad/0.02s

% Weights
params.q_y  = 200;
params.q_vy = 0;
params.q_psi= 0;
params.q_r  = 1;
params.q_st = 0;
params.q_dst= 0;

params.p_y  = 1000;
params.p_vy = 0;
params.p_psi= 0;
params.p_r  = 0;
params.p_st = 0;
params.p_dst= 0;

params.r_st = 1;

params.rd_st = 2;

% Bounds
params.min_st = -0.38;
params.max_st = 0.38;
params.max_delta = 0.45; % [rad], physical steering-position limit
params.max_st_rate = 1.396; % [rad/s], command and actuator-state limit

% Debug options
debug_opts.enabled = false;
debug_opts.step    = [];     % [] = all, or e.g. 20
debug_opts.pause   = false ;  % true = step-by-step
debug_opts.figure_id = 99;

% Compare options
comp_opts.enabled = false;
comp_opts.step    = [];     % [] = all, or e.g. 20
comp_opts.pause   = false;  % true = step-by-step
comp_opts.figure_id = 98;

%% SIMULATE

% Global state history: [x y psi vx vy r delta delata_dot]
X = cell(n,1);
U = cell(n,1);
model_error = NaN(1,n);
solver_exitflag = NaN(1,n-1);
mpc_compute_time = NaN(1,n-1);
model_matrix_compute_time = NaN(1,n-1);
solver_compute_time = NaN(1,n-1);
adaptation_compute_time = zeros(1,n-1);

% Initial GLOBAL state. Use only samples available at the initial instant.
if idx_start > 1
    initial_delta_dot = (data.delta(idx_start)-data.delta(idx_start-1))/dt;
else
    initial_delta_dot = 0;
end
X{1} = [meas.x(1);
        meas.y(1);
        meas.psi(1);
        meas.vx(1);
        meas.vy(1);
        meas.r(1);
        meas.delta(1);
        initial_delta_dot];

U{1} = in.st(1);

% Warm start. The state guess is initialized after the first local frame is
% known, then shifted after every solve.
x_pred = zeros(Np,nx);
u_pred = repmat(in.st(1),Np,1);
reference_idx = [];
reference_idx_history = NaN(n,1);
x_ref = zeros(Np,nx);
vx_ref = zeros(Np,1);
if strcmpi(params.model,"eefig")
    eefig_reset();
end

for k = 1:n-1

    % Current global state
    Xg = X{k};

    % Find global reference
    [X_ref_global,reference_idx] = build_reference_global( ...
        traj,Xg,dt,Np+1,reference_idx);
    reference_idx_history(k) = reference_idx;

    % Convert GLOBAL → LOCAL MPC state
    [x_0, ~] = global_to_local_state(Xg, X_ref_global{1});
    if k == 1
        x_pred = repmat(x_0.',Np,1);
    end
    for i = 1:Np
        [x_ref(i,:), vx_ref(i)] = global_to_local_state(X_ref_global{i+1}, X_ref_global{1});
    end

    % MPC solve
    x_ref_vec  = reshape(x_ref.', [], 1);
    x_pred_vec = reshape(x_pred.', [], 1);
    u_pred_vec = reshape(u_pred.', [], 1);

    mpc_timer = tic;
    [x_pred_vec,u_pred_vec,~,mpc_info] = mpc( ...
        x_0,x_ref_vec,x_pred_vec,u_pred_vec,vx_ref,U{k},params);
    mpc_compute_time(k) = toc(mpc_timer);
    model_matrix_compute_time(k) = mpc_info.model_matrix_time;
    solver_compute_time(k) = mpc_info.solver_time;
    solver_exitflag(k) = mpc_info.exitflag;

    x_solution = reshape(x_pred_vec,nx,[]).';
    u_solution = reshape(u_pred_vec,nu,[]).';

    % Debug plots 
    mpc_debug_plot(k,x_0,x_ref,x_solution,u_solution,params,debug_opts);

    % Apply first control
    u = u_solution(1,:)';
    % u = in.st(k); % Test with measured steering

    U{k+1} = u;

    % Simulate GLOBAL dynamics
    % X{k+1} = sim_anfis_delta(Xg', u', vx_ref(1), dt)';
    X{k+1} = sim_bicycleDynamic_linear(Xg',u',meas.vx(k+1),dt)';
    % X{k+1} = sim_ltv(Xg', u', vx_ref(1), dt);

    if strcmpi(params.model,"eefig") && params.eefig_adaptive
        adaptation_timer = tic;
        x_measured = [0;Xg(5);0;Xg(6);Xg(7);Xg(8)];
        x_next_measured = [0;X{k+1}(5);0;X{k+1}(6); ...
            X{k+1}(7);X{k+1}(8)];
        eefig_online_update(x_measured,[Xg(4);u],x_next_measured);
        adaptation_compute_time(k) = toc(adaptation_timer);
    end

    % Compare last seen states with mpc predicted. Model error
    if k > Np+1
        s = k-Np+1;
        X_compare = X(s:k+1);
        U_compare = U(s+1:k+1);
        
        % Convert to local
        x_comp = NaN(Np,6);
        u_comp = NaN(Np,1);
        vx_comp = NaN(Np,1);
        % x_0_comp = global_to_local_state(X_compare{1}, X_compare{1});
        % for i = 1:Np
        %     [x_comp(i,:), ~] = global_to_local_state(X_compare{i+1}, X_compare{1});
        %     u_comp(i,:) = U_compare{i}';
        %     vx_comp(i) = X_compare{i}(4);
        % end
        x_0_comp = X_compare{1}([2 5 3 6 7 8]);
        
        for i = 1:Np
            x_comp(i,:) = X_compare{i+1}([2 5 3 6 7 8]);
            u_comp(i,:) = U_compare{i}';
            vx_comp(i)  = X_compare{i}(4);
        end

        % Use MPC function again to compute the predicted with 
        x_comp_vec  = reshape(x_comp.', [], 1);
        u_comp_vec  = reshape(u_comp.', [], 1);

        comparison_params = params;
        comparison_params.solve_problem = false;
        [~,~,x_pred_comp_vec] = mpc(x_0_comp,x_comp_vec,x_comp_vec, ...
            u_comp_vec,vx_comp,U{k},comparison_params);

        x_pred_comp = reshape(x_pred_comp_vec, nx, []).';

        % Norm difference
        err = reshape(x_pred_comp_vec-x_comp_vec,nx,[]);

        err(1,:) = err(1,:) / params.scale_y;
        err(2,:) = err(2,:) / params.scale_vy;
        err(3,:) = err(3,:) / params.scale_psi;
        err(4,:) = err(4,:) / params.scale_r;
        
        model_error(k) = mean(vecnorm(err(1:4,:),2,1));
        fprintf("Model error: %.6f\n", model_error(k));

        % Comparation plots
        mpc_debug_plot(k, x_0_comp, x_comp, x_pred_comp, u_comp, params, comp_opts);
    end

    % Shift the solution so the next LTV/TS linearization is centered on
    % the same future instants as the next receding horizon.
    x_pred = [x_solution(2:end,:);x_solution(end,:)];
    u_pred = [u_solution(2:end,:);u_solution(end,:)];

    fprintf("Iteration %i done\n", k);
end

% Associate the final state with the next monotonic path point.
reference_idx_history(n) = find_closest_point( ...
    traj,X{n}(1),X{n}(2),reference_idx);
%% PLOT RESULTS

% Convert cell → matrix
Xg_mat = cell2mat(X')';   % N x 8
U_mat  = cell2mat(U')';   % N x 1

% Extract global states
x_sim = Xg_mat(:,1);
y_sim = Xg_mat(:,2);
delta_sim = Xg_mat(:,7);

st_sim = U_mat(:,1);

% Signed cross-track error in the local frame of the path reference used
% by the controller. Positive error points to the left of the path tangent.
ref_x = traj.x(reference_idx_history);
ref_y = traj.y(reference_idx_history);
ref_psi = traj.psi(reference_idx_history);
lateral_error = -sin(ref_psi).*(x_sim-ref_x) ...
    + cos(ref_psi).*(y_sim-ref_y);

%t_u = meas.t(1:size(U_mat,1));
t_u = 0:dt:dt*(size(U_mat,1)-1);

figure('Name','MPC Results','Position',[100 100 1200 800]);
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

% ===== TOP: GLOBAL TRAJECTORY =====
ax1 = nexttile; hold on; grid on; axis equal;

% Reference path from measurements (black solid)
plot(meas.x,meas.y,'k-','LineWidth',2);

% Simulated path (white dashed)
plot(x_sim, y_sim, '-r', 'LineWidth', 2);

xlabel('X [m]');
ylabel('Y [m]');
title('Global Trajectory Tracking');

legend('Reference (meas)','Simulated','Location','best');

set(ax1,'Color','w');

% ===== MIDDLE: LATERAL TRACKING ERROR =====
ax2 = nexttile; hold on; grid on;

plot(t_u,lateral_error,'k-','LineWidth',1.5);
yline(0,'Color',[0.5 0.5 0.5],'LineStyle','--', ...
    'HandleVisibility','off');
ylabel('e_y [m]');
xlabel('Time [s]');
title(sprintf('Signed lateral error: RMSE %.3f m, max |e_y| %.3f m', ...
    sqrt(mean(lateral_error.^2)),max(abs(lateral_error))));

% ===== BOTTOM: MODEL ERROR =====
ax3 = nexttile; hold on; grid on;

plot(t_u, model_error, 'LineWidth', 1.5);
ylabel('Vectorn norm difference');

xlabel('Time [s]');
title('Model Error');
legend('Model error','Location','best');

linkaxes([ax2 ax3],'x')

%% PERFORMANCE INSIGHTS

valid_model_error = model_error(isfinite(model_error));
if isempty(valid_model_error)
    mean_model_error = NaN;
    model_error_p95 = NaN;
else
    mean_model_error = mean(valid_model_error);
    model_error_p95 = prctile(valid_model_error,95);
end
command_rate = diff(st_sim)/dt;
online_cycle_time = mpc_compute_time+adaptation_compute_time;
model_label = string(params.model);
if strcmpi(params.model,"eefig")
    if params.eefig_adaptive
        model_label = model_label+" adaptive";
    else
        model_label = model_label+" frozen";
    end
end
mpc_insights = table( ...
    model_label,evaluation_run,string(selected_run.event), ...
    string(selected_run.track_layout),(n-1)*dt, ...
    mean(lateral_error),mean(abs(lateral_error)), ...
    sqrt(mean(lateral_error.^2)),prctile(abs(lateral_error),95), ...
    max(abs(lateral_error)),mean_model_error,model_error_p95, ...
    1e3*mean(model_matrix_compute_time), ...
    1e3*prctile(model_matrix_compute_time,95), ...
    1e3*mean(solver_compute_time), ...
    1e3*prctile(solver_compute_time,95), ...
    1e3*mean(mpc_compute_time),1e3*prctile(mpc_compute_time,95), ...
    1e3*mean(adaptation_compute_time), ...
    1e3*prctile(adaptation_compute_time,95), ...
    1e3*mean(online_cycle_time),1e3*prctile(online_cycle_time,95), ...
    1e3*max(online_cycle_time),100*mean(online_cycle_time>dt), ...
    sum(solver_exitflag~=1),sqrt(mean(st_sim.^2)), ...
    sqrt(mean(command_rate.^2)),max(abs(command_rate)), ...
    'VariableNames',{ ...
    'Model','EvaluationRun','Event','TrackLayout','Duration_s', ...
    'MeanSignedLateralError_m','LateralMAE_m','LateralRMSE_m', ...
    'LateralP95_m','LateralMaxAbs_m','MeanModelError', ...
    'ModelErrorP95','MeanModelMatrixTime_ms','ModelMatrixTimeP95_ms', ...
    'MeanSolverTime_ms','SolverTimeP95_ms','MeanMPCTime_ms', ...
    'MPCTimeP95_ms','MeanAdaptationTime_ms','AdaptationTimeP95_ms', ...
    'MeanOnlineCycleTime_ms','OnlineCycleTimeP95_ms', ...
    'MaxOnlineCycleTime_ms','DeadlineMissPercent', ...
    'SolverFailures','SteeringCommandRMS_rad', ...
    'SteeringCommandRateRMS_rad_s','SteeringCommandRateMax_rad_s'});

fprintf('\nMPC PERFORMANCE INSIGHTS\n');
disp(mpc_insights);


%%








%% AUXILIAR FUNCTIONS

function psi_path = compute_path_heading(x, y)
%COMPUTE_PATH_HEADING Estimate heading of the path from x-y trajectory

    dx = gradient(x);
    dy = gradient(y);

    psi_path = unwrap(atan2(dy, dx));
end

function idx = find_closest_point(traj,x,y,previous_idx)

    if isempty(previous_idx)
        candidates = 1:numel(traj.s);
    else
        % Preserve path progress and restrict the search to the next 5 m.
        search_samples = max(1,ceil(5/median(diff(traj.s))));
        last_idx = min(numel(traj.s),previous_idx+search_samples);
        candidates = previous_idx:last_idx;
    end
    dx = traj.x(candidates)-x;
    dy = traj.y(candidates)-y;

    [~,local_idx] = min(dx.^2+dy.^2);
    idx = candidates(local_idx);
end

function [X_ref,idx] = build_reference_global( ...
        traj,Xg,dt,Np,previous_idx)
% Build reference state list in global coordinates
% Global state: [x y psi vx vy r delta delta_dot]

    X_ref = cell(Np,1);
    
    % Find point closest to actual position
    idx = find_closest_point(traj,Xg(1),Xg(2),previous_idx);

    s_i = traj.s(idx);
    for i = 1:Np

        if s_i > traj.s(end)
            s_i = traj.s(end);
        end

        % Use trajectory speed
        vx_i = interp1(traj.s, traj.vx, s_i, 'linear');

        % Interpolate trajectory
        x_t   = interp1(traj.s, traj.x, s_i, 'spline');
        y_t   = interp1(traj.s, traj.y, s_i, 'spline');
        psi_t = interp1(traj.s, traj.psi, s_i, 'spline');
        kappa_t = interp1(traj.s,traj.kappa,s_i,'linear');
        r_t = vx_i*kappa_t;

        % Propagate arc-length using trajectory speed
        s_i = s_i + vx_i * dt;

        % Build global reference states
        X_ref{i} = [x_t; y_t; psi_t; vx_i; 0; r_t; 0; 0];
    end
end

function [x_local, vx] = global_to_local_state(X_global, X_ref)
% Convert global state into local frame defined by X_ref
% Global state: [x y psi vx vy r delta delta_dot]
% Local state:  [y vy psi r delta delta_dot]

    % Extract global state
    x = X_global(1);
    y = X_global(2);
    psi = X_global(3);
    vx = X_global(4);
    vy = X_global(5);
    r = X_global(6);
    delta = X_global(7);
    delta_dot = X_global(8);

    % Extract reference state
    x_ref = X_ref(1);
    y_ref = X_ref(2);
    psi_ref = X_ref(3);

    % Relative position
    dx = x - x_ref;
    dy = y - y_ref;

    % Rotation: global → local (reference frame)
    y_local_pos = -sin(psi_ref)*dx + cos(psi_ref)*dy;

    % Relative heading
    psi_rel = wrapToPi(psi - psi_ref);

    % Assemble output
    x_local = zeros(6,1);
    x_local(1) = y_local_pos;
    x_local(2) = vy;
    x_local(3) = psi_rel;
    x_local(4) = r;
    x_local(5) = delta;
    x_local(6) = delta_dot;

end
