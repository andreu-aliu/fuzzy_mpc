function result = run_mpc_scenario(selected_run,evaluation_run, ...
        dataset_file,cfg,params,plant_model,debug_opts,comp_opts)
%RUN_MPC_SCENARIO Prepare and simulate one continuous closed-loop scenario.

data = selected_run.data;
dt = cfg.dt;
Np = cfg.Np;
nx = cfg.nx;
nu = cfg.nu;

if cfg.use_full_run
    idx_start = 1+round(cfg.trim_start_seconds/dt);
    idx_end = numel(data.vx)-round(cfg.trim_end_seconds/dt);
    if isfinite(cfg.max_mpc_windows_per_run)
        % N samples contain N-1 consecutive closed-loop MPC windows.
        idx_end = min(idx_end,idx_start+cfg.max_mpc_windows_per_run);
    end
else
    idx_start = cfg.idx_start;
    idx_end = min(idx_start+cfg.window_sample_count-1,numel(data.vx));
end

assert(idx_start>=1 && idx_end>idx_start && idx_end<=numel(data.vx), ...
    'Run %d has no valid interval for the configured window.',evaluation_run);

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
    meas.psi = unwrap(meas.course-atan2(meas.vy,meas.vx));
end
n = numel(meas.t);

required = [meas.t(:);meas.x(:);meas.y(:);meas.vx(:);meas.vy(:); ...
    meas.r(:);meas.delta(:);meas.psi(:);in.st(:)];
assert(all(isfinite(required)), ...
    'Evaluation run %d contains non-finite values in its selected interval.', ...
    evaluation_run);

traj = build_trajectory(meas,cfg.path_spacing);

fprintf('Evaluation run %d: %s | %s | %g lap(s) | %d samples\n', ...
    evaluation_run,string(selected_run.event), ...
    string(selected_run.track_layout),selected_run.laps,numel(data.time));
fprintf('Run %d will evaluate %d consecutive MPC window(s).\n', ...
    evaluation_run,n-1);

setup = build_setup(dataset_file,evaluation_run,idx_start,n,cfg,params, ...
    plant_model);

% Global state: [x y psi vx vy r delta delta_dot].
X = cell(n,1);
U = cell(n,1);
model_error = NaN(1,n);
solver_exitflag = NaN(1,n-1);
mpc_compute_time = NaN(1,n-1);
model_matrix_compute_time = NaN(1,n-1);
solver_compute_time = NaN(1,n-1);
adaptation_compute_time = zeros(1,n-1);

if idx_start>1
    initial_delta_dot = ...
        (data.delta(idx_start)-data.delta(idx_start-1))/dt;
else
    initial_delta_dot = 0;
end
X{1} = [meas.x(1);meas.y(1);meas.psi(1);meas.vx(1);meas.vy(1); ...
    meas.r(1);meas.delta(1);initial_delta_dot];
U{1} = in.st(1);

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
    Xg = X{k};
    [X_ref_global,reference_idx] = build_reference_global( ...
        traj,Xg,dt,Np+1,reference_idx);
    reference_idx_history(k) = reference_idx;

    [x_0,~] = global_to_local_state(Xg,X_ref_global{1});
    if k==1
        x_pred = repmat(x_0.',Np,1);
    end
    for i = 1:Np
        [x_ref(i,:),vx_ref(i)] = global_to_local_state( ...
            X_ref_global{i+1},X_ref_global{1});
    end

    x_ref_vec = reshape(x_ref.',[],1);
    x_pred_vec = reshape(x_pred.',[],1);
    u_pred_vec = reshape(u_pred.',[],1);
    mpc_timer = tic;
    [x_pred_vec,u_pred_vec,~,mpc_info] = mpc( ...
        x_0,x_ref_vec,x_pred_vec,u_pred_vec,vx_ref,U{k},params);
    mpc_compute_time(k) = toc(mpc_timer);
    model_matrix_compute_time(k) = mpc_info.model_matrix_time;
    solver_compute_time(k) = mpc_info.solver_time;
    solver_exitflag(k) = mpc_info.exitflag;

    x_solution = reshape(x_pred_vec,nx,[]).';
    u_solution = reshape(u_pred_vec,nu,[]).';
    mpc_debug_plot(k,x_0,x_ref,x_solution,u_solution,params,debug_opts);

    u = u_solution(1,:).';
    U{k+1} = u;
    switch plant_model
        case "nonlinear_bicycle_linear_tire"
            X{k+1} = sim_bicycleDynamic_linear( ...
                Xg.',u.',meas.vx(k+1),dt).';
        case "nonlinear_bicycle"
            X{k+1} = sim_nonlinear_bicycle( ...
                Xg.',u.',meas.vx(k+1),dt).';
        case "anfis_delta"
            X{k+1} = sim_anfis_delta(Xg.',u.',meas.vx(k+1),dt).';
        case "ltv"
            X{k+1} = sim_ltv(Xg,u,meas.vx(k+1),dt);
        otherwise
            error('Unknown simulation plant "%s".',plant_model);
    end

    if strcmpi(params.model,"eefig") && params.eefig_adaptive
        adaptation_timer = tic;
        x_measured = [0;Xg(5);0;Xg(6);Xg(7);Xg(8)];
        x_next_measured = [0;X{k+1}(5);0;X{k+1}(6); ...
            X{k+1}(7);X{k+1}(8)];
        eefig_online_update(x_measured,[Xg(4);u],x_next_measured);
        adaptation_compute_time(k) = toc(adaptation_timer);
    end

    if k>Np+1
        first_idx = k-Np+1;
        X_compare = X(first_idx:k+1);
        U_compare = U(first_idx+1:k+1);
        x_comp = NaN(Np,nx);
        u_comp = NaN(Np,nu);
        vx_comp = NaN(Np,1);
        x_0_comp = X_compare{1}([2 5 3 6 7 8]);
        for i = 1:Np
            x_comp(i,:) = X_compare{i+1}([2 5 3 6 7 8]);
            u_comp(i,:) = U_compare{i}.';
            vx_comp(i) = X_compare{i}(4);
        end
        x_comp_vec = reshape(x_comp.',[],1);
        u_comp_vec = reshape(u_comp.',[],1);
        comparison_params = params;
        comparison_params.solve_problem = false;
        [~,~,x_pred_comp_vec] = mpc(x_0_comp,x_comp_vec,x_comp_vec, ...
            u_comp_vec,vx_comp,U{k},comparison_params);
        x_pred_comp = reshape(x_pred_comp_vec,nx,[]).';
        err = reshape(x_pred_comp_vec-x_comp_vec,nx,[]);
        err(1,:) = err(1,:)/params.scale_y;
        err(2,:) = err(2,:)/params.scale_vy;
        err(3,:) = err(3,:)/params.scale_psi;
        err(4,:) = err(4,:)/params.scale_r;
        model_error(k) = mean(vecnorm(err(1:4,:),2,1));
        mpc_debug_plot(k,x_0_comp,x_comp,x_pred_comp,u_comp, ...
            params,comp_opts);
    end

    x_pred = [x_solution(2:end,:);x_solution(end,:)];
    u_pred = [u_solution(2:end,:);u_solution(end,:)];
    if ~cfg.is_batch || mod(k,100)==0 || k==n-1
        fprintf('Run %d: iteration %d/%d done\n',evaluation_run,k,n-1);
    end
end

reference_idx_history(n) = find_closest_point( ...
    traj,X{n}(1),X{n}(2),reference_idx);
Xg_mat = cell2mat(X.').';
U_mat = cell2mat(U.').';
x_sim = Xg_mat(:,1);
y_sim = Xg_mat(:,2);
st_sim = U_mat(:,1);
ref_x = traj.x(reference_idx_history);
ref_y = traj.y(reference_idx_history);
ref_psi = traj.psi(reference_idx_history);
lateral_error = -sin(ref_psi).*(x_sim-ref_x) ...
    + cos(ref_psi).*(y_sim-ref_y);
t = (0:n-1).'*dt;

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
    model_label = model_label+string(ternary(params.eefig_adaptive, ...
        ' adaptive',' frozen'));
end

insights = table(model_label,evaluation_run,string(selected_run.event), ...
    string(selected_run.track_layout),double(selected_run.laps),(n-1)*dt, ...
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
    'VariableNames',{'Model','EvaluationRun','Event','TrackLayout', ...
    'DeclaredLaps','Duration_s','MeanSignedLateralError_m','LateralMAE_m', ...
    'LateralRMSE_m','LateralP95_m','LateralMaxAbs_m','MeanModelError', ...
    'ModelErrorP95','MeanModelMatrixTime_ms','ModelMatrixTimeP95_ms', ...
    'MeanSolverTime_ms','SolverTimeP95_ms','MeanMPCTime_ms', ...
    'MPCTimeP95_ms','MeanAdaptationTime_ms','AdaptationTimeP95_ms', ...
    'MeanOnlineCycleTime_ms','OnlineCycleTimeP95_ms', ...
    'MaxOnlineCycleTime_ms','DeadlineMissPercent','SolverFailures', ...
    'SteeringCommandRMS_rad','SteeringCommandRateRMS_rad_s', ...
    'SteeringCommandRateMax_rad_s'});

result = struct('evaluation_run',evaluation_run,'selected_run',selected_run, ...
    'data',data,'meas',meas,'traj',traj,'x_sim',x_sim,'y_sim',y_sim, ...
    'st_sim',st_sim,'model_error',model_error,'lateral_error', ...
    lateral_error,'time',t,'setup',setup,'insights',insights);
end

function setup = build_setup(dataset_file,evaluation_run,idx_start,n,cfg, ...
        params,plant_model)
setup = struct();
setup.DatasetFile = char(dataset_file);
setup.EvaluationRun = evaluation_run;
setup.WindowStart = idx_start;
setup.SampleCount = n;
if cfg.use_full_run && isinf(cfg.max_mpc_windows_per_run)
    setup.EvaluationProtocol = 'full_run';
elseif cfg.use_full_run
    setup.EvaluationProtocol = 'capped_run';
else
    setup.EvaluationProtocol = 'selected_window';
end
setup.TrimStartSeconds = double(cfg.trim_start_seconds);
setup.TrimEndSeconds = double(cfg.trim_end_seconds);
if cfg.use_full_run
    setup.MaxMPCWindowsPerRun = double(cfg.max_mpc_windows_per_run);
else
    setup.MaxMPCWindowsPerRun = NaN;
end
setup.PredictionModel = char(params.model);
setup.EEFigAdaptive = logical(strcmpi(params.model,"eefig") ...
    && params.eefig_adaptive);
setup.PlantModel = char(plant_model);
setup.HorizonSteps = cfg.Np;
setup.Ts = cfg.dt;
setup.Scales = [params.scale_y params.scale_vy params.scale_psi ...
    params.scale_r params.scale_st params.scale_dst];
setup.RunningWeights = [params.q_y params.q_vy params.q_psi ...
    params.q_r params.q_st params.q_dst];
setup.TerminalWeights = [params.p_y params.p_vy params.p_psi ...
    params.p_r params.p_st params.p_dst];
setup.ControlWeights = [params.r_st params.rd_st];
setup.Limits = [params.min_st params.max_st params.max_delta ...
    params.max_st_rate];
setup.ReferenceMethod = ...
    'measured_xy_with_curvature_yaw_rate_nonnegative_progress';
end

function traj = build_trajectory(meas,ds)
s = [0;cumsum(hypot(diff(meas.x),diff(meas.y)))];
[s,unique_idx] = unique(s,'stable');
assert(numel(s)>=2,'The selected trajectory has no usable arc length.');
s_uniform = (0:ds:s(end)).';
assert(numel(s_uniform)>=2,'The selected trajectory is shorter than ds.');
traj.s = s_uniform;
traj.x = interp1(s,meas.x(unique_idx),s_uniform,'pchip');
traj.y = interp1(s,meas.y(unique_idx),s_uniform,'pchip');
traj.vx = interp1(s,meas.vx(unique_idx),s_uniform,'linear');
traj.psi = unwrap(atan2(gradient(traj.y,ds),gradient(traj.x,ds)));
traj.kappa = gradient(traj.psi,ds);
assert(all(isfinite([traj.s;traj.x;traj.y;traj.vx; ...
    traj.psi;traj.kappa])),'Interpolated trajectory is not finite.');
end

function psi_path = compute_path_heading(x,y)
psi_path = unwrap(atan2(gradient(y),gradient(x)));
end

function idx = find_closest_point(traj,x,y,previous_idx)
if isempty(previous_idx)
    candidates = 1:numel(traj.s);
else
    search_samples = max(1,ceil(5/median(diff(traj.s))));
    last_idx = min(numel(traj.s),previous_idx+search_samples);
    candidates = previous_idx:last_idx;
end
[~,local_idx] = min((traj.x(candidates)-x).^2+ ...
    (traj.y(candidates)-y).^2);
idx = candidates(local_idx);
end

function [X_ref,idx] = build_reference_global(traj,Xg,dt,Np,previous_idx)
X_ref = cell(Np,1);
idx = find_closest_point(traj,Xg(1),Xg(2),previous_idx);
s_i = traj.s(idx);
for i = 1:Np
    s_i = min(max(s_i,traj.s(1)),traj.s(end));
    vx_i = max(interp1(traj.s,traj.vx,s_i,'linear'),0);
    x_i = interp1(traj.s,traj.x,s_i,'spline');
    y_i = interp1(traj.s,traj.y,s_i,'spline');
    psi_i = interp1(traj.s,traj.psi,s_i,'spline');
    kappa_i = interp1(traj.s,traj.kappa,s_i,'linear');
    X_ref{i} = [x_i;y_i;psi_i;vx_i;0;vx_i*kappa_i;0;0];
    s_i = min(s_i+vx_i*dt,traj.s(end));
end
end

function [x_local,vx] = global_to_local_state(X_global,X_ref)
dx = X_global(1)-X_ref(1);
dy = X_global(2)-X_ref(2);
vx = X_global(4);
x_local = [ ...
    -sin(X_ref(3))*dx+cos(X_ref(3))*dy; ...
    X_global(5);wrapToPi(X_global(3)-X_ref(3));X_global(6); ...
    X_global(7);X_global(8)];
end

function value = ternary(condition,true_value,false_value)
if condition
    value = true_value;
else
    value = false_value;
end
end
