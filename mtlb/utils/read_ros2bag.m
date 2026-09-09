function [data, audit] = read_ros2bag(bagPath, Ts)

% READ_ROS2_CAR_DATA
% Reads ROS2 bag and returns synchronized signals on a uniform time grid.
% The optional second output contains raw timing and interpolation diagnostics.

validateattributes(Ts, {'numeric'}, {'scalar','real','finite','positive'}, ...
    mfilename, 'Ts');
want_audit = nargout > 1;

% Open bag
bag = ros2bagreader(bagPath);

% STATE TOPIC (REFERENCE)
stateSel  = select(bag,"Topic","/as/c/state");
stateMsgs = readMessages(stateSel);
t_state = stateSel.MessageList.Time;

assert_topic_not_empty(t_state, '/as/c/state');

Ns = numel(stateMsgs);

x  = zeros(Ns,1);
y  = zeros(Ns,1);
psi = zeros(Ns,1);
vx = zeros(Ns,1);
vy = zeros(Ns,1);
r  = zeros(Ns,1);
ax = zeros(Ns,1);
ay = zeros(Ns,1);

for i = 1:Ns
    msg = stateMsgs{i};

    x(i)  = msg.odom.position.x;
    y(i)  = msg.odom.position.y;
    % odom.heading is the scalar planar yaw used by the live controllers.
    % Project it onto the wrapped 2-D angle before unwrapping the sequence;
    % this avoids interpolating across the +/-pi discontinuity.
    psi(i) = atan2(sin(double(msg.odom.heading)), ...
        cos(double(msg.odom.heading)));
    vx(i) = msg.odom.velocity.x;
    vy(i) = msg.odom.velocity.y;
    r(i)  = msg.odom.velocity.w;
    ax(i) = msg.odom.acceleration.x;
    ay(i) = msg.odom.acceleration.y;
end

% STEERING
steerSel  = select(bag,"Topic","/el/sensor/driver_inputs");
steerMsgs = readMessages(steerSel);
t_steer = steerSel.MessageList.Time;

assert_topic_not_empty(t_steer, '/el/sensor/driver_inputs');

Nst = numel(steerMsgs);

steering = zeros(Nst,1);

for i = 1:Nst
    msg = steerMsgs{i};

    steering(i) = msg.steering;
end

% STEERING CMD
steerCmdSel  = select(bag,"Topic","/as/c/steering");
steerCmdMsgs = readMessages(steerCmdSel);
t_steerCmd = steerCmdSel.MessageList.Time;

assert_topic_not_empty(t_steerCmd, '/as/c/steering');

NstCmd = numel(steerCmdMsgs);

steeringCmd = zeros(NstCmd,1);

for i = 1:NstCmd
    msg = steerCmdMsgs{i};

    steeringCmd(i) = msg.steering;
end

% TORQUE VECTORING
tvSel  = select(bag,"Topic","/ctrl/llc/torque_vectoring");
tvMsgs = readMessages(tvSel);
t_tv = tvSel.MessageList.Time;

assert_topic_not_empty(t_tv, '/ctrl/llc/torque_vectoring');

Nt = numel(tvMsgs);

mz  = zeros(Nt,1);
Tfl = zeros(Nt,1);
Tfr = zeros(Nt,1);
Trl = zeros(Nt,1);
Trr = zeros(Nt,1);

for i = 1:Nt
    msg = tvMsgs{i};

    mz(i)  = msg.actual_mz;
    Tfl(i) = msg.front_left_torque;
    Tfr(i) = msg.front_right_torque;
    Trl(i) = msg.rear_left_torque;
    Trr(i) = msg.rear_right_torque;
end

% Record raw timing quality before duplicates are removed.
if want_audit
    audit.Ts = Ts;
    audit.timing.state = timestamp_stats(t_state, Ts);
    audit.timing.steering = timestamp_stats(t_steer, Ts);
    audit.timing.steering_command = timestamp_stats(t_steerCmd, Ts);
    audit.timing.torque_vectoring = timestamp_stats(t_tv, Ts);
end

assert_monotonic_time(t_state, '/as/c/state');
assert_monotonic_time(t_steer, '/el/sensor/driver_inputs');
assert_monotonic_time(t_steerCmd, '/as/c/steering');
assert_monotonic_time(t_tv, '/ctrl/llc/torque_vectoring');

% REMOVE DUPLICATES
t0 = t_state(1);

t_state    = t_state - t0;
t_steer    = t_steer - t0;
t_steerCmd = t_steerCmd - t0;
t_tv       = t_tv - t0;

[t_state_unique, idx_state] = unique(t_state, 'stable');
x_unique = x(idx_state);
y_unique = y(idx_state);
psi_unique = unwrap(psi(idx_state));
vx_unique = vx(idx_state);
vy_unique = vy(idx_state);
r_unique = r(idx_state);
ax_unique = ax(idx_state);
ay_unique = ay(idx_state);

[t_steer_unique, idx_steer] = unique(t_steer, 'stable');
steering_unique = steering(idx_steer);

[t_steerCmd_unique, idx_steerCmd] = unique(t_steerCmd, 'stable');
steerCmd_unique = steeringCmd(idx_steerCmd);

[t_tv_unique, idx_tv] = unique(t_tv, 'stable');

mz_unique  = mz(idx_tv);
Tfl_unique = Tfl(idx_tv);
Tfr_unique = Tfr(idx_tv);
Trl_unique = Trl(idx_tv);
Trr_unique = Trr(idx_tv);

% RESAMPLING
t_start = max([t_state(1), t_steer_unique(1), t_steerCmd_unique(1), t_tv_unique(1)]);
t_end   = min([t_state(end), t_steer_unique(end), t_steerCmd_unique(end), t_tv_unique(end)]);

t_uniform = (t_start:Ts:t_end)';

x_100  = interp1(t_state_unique, x_unique,  t_uniform, 'linear');
y_100  = interp1(t_state_unique, y_unique,  t_uniform, 'linear');
psi_100 = interp1(t_state_unique, psi_unique, t_uniform, 'linear');
vx_100 = interp1(t_state_unique, vx_unique, t_uniform, 'linear');
vy_100 = interp1(t_state_unique, vy_unique, t_uniform, 'linear');
r_100  = interp1(t_state_unique, r_unique,  t_uniform, 'linear');
ax_100 = interp1(t_state_unique, ax_unique, t_uniform, 'linear');
ay_100 = interp1(t_state_unique, ay_unique, t_uniform, 'linear');

steering_100 = interp1(t_steer_unique, steering_unique, ...
                       t_uniform, 'linear');

steeringCmd_100 = interp1(t_steerCmd_unique, steerCmd_unique, ...
                       t_uniform, 'linear');

mz_100  = interp1(t_tv_unique, mz_unique,  t_uniform, 'linear');
Tfl_100 = interp1(t_tv_unique, Tfl_unique, t_uniform, 'linear');
Tfr_100 = interp1(t_tv_unique, Tfr_unique, t_uniform, 'linear');
Trl_100 = interp1(t_tv_unique, Trl_unique, t_uniform, 'linear');
Trr_100 = interp1(t_tv_unique, Trr_unique, t_uniform, 'linear');

% TRIM STANDSTILL (START/END) USING VX
% Keeps the central segment where the vehicle is moving.
% Tune these two if needed for your data/noise level.
vx_trim_threshold = 0.20;   % [m/s]
trim_confirm_time = 0.50;   % [s] required moving time

[keepStart, keepEnd] = vx_moving_window(vx_100, Ts, vx_trim_threshold, trim_confirm_time);

t_uniform_absolute = t_uniform(keepStart:keepEnd);
trim_time_origin = t_uniform_absolute(1);
t_uniform = t_uniform(keepStart:keepEnd);
t_uniform = t_uniform - t_uniform(1);

x_100  = x_100(keepStart:keepEnd);
y_100  = y_100(keepStart:keepEnd);
psi_100 = psi_100(keepStart:keepEnd);
vx_100 = vx_100(keepStart:keepEnd);
vy_100 = vy_100(keepStart:keepEnd);
r_100  = r_100(keepStart:keepEnd);
ax_100 = ax_100(keepStart:keepEnd);
ay_100 = ay_100(keepStart:keepEnd);

steering_100 = steering_100(keepStart:keepEnd);

steeringCmd_100 = steeringCmd_100(keepStart:keepEnd);

mz_100  = mz_100(keepStart:keepEnd);
Tfl_100 = Tfl_100(keepStart:keepEnd);
Tfr_100 = Tfr_100(keepStart:keepEnd);
Trl_100 = Trl_100(keepStart:keepEnd);
Trr_100 = Trr_100(keepStart:keepEnd);

% CREATE TIMESERIES
data.time = t_uniform;

data.x  = x_100;
data.y  = y_100;
data.psi = psi_100;
data.vx = vx_100;
data.vy = vy_100;
data.r  = r_100;
data.ax = ax_100;
data.ay = ay_100;

data.delta = steering_100;

data.st = steeringCmd_100;

data.mz  = mz_100;
data.Tfl = Tfl_100;
data.Tfr = Tfr_100;
data.Trl = Trl_100;
data.Trr = Trr_100;

if want_audit
    % Times use the same zero as data.time.
    audit.raw.state.time = t_state_unique - trim_time_origin;
    audit.raw.state.x = x_unique;
    audit.raw.state.y = y_unique;
    audit.raw.state.psi = psi_unique;
    audit.raw.state.vx = vx_unique;
    audit.raw.state.vy = vy_unique;
    audit.raw.state.r = r_unique;
    audit.raw.state.ax = ax_unique;
    audit.raw.state.ay = ay_unique;

    audit.raw.steering.time = t_steer_unique - trim_time_origin;
    audit.raw.steering.value = steering_unique;
    audit.raw.steering_command.time = t_steerCmd_unique - trim_time_origin;
    audit.raw.steering_command.value = steerCmd_unique;
    audit.raw.torque_vectoring.time = t_tv_unique - trim_time_origin;
    audit.raw.torque_vectoring.mz = mz_unique;
    audit.raw.torque_vectoring.Tfl = Tfl_unique;
    audit.raw.torque_vectoring.Tfr = Tfr_unique;
    audit.raw.torque_vectoring.Trl = Trl_unique;
    audit.raw.torque_vectoring.Trr = Trr_unique;

    audit.uniform_time = data.time;
    audit.trim.original_start = trim_time_origin;
    audit.trim.keep_start = keepStart;
    audit.trim.keep_end = keepEnd;
    audit.nearest_distance.state = nearest_sample_distance( ...
        t_uniform_absolute, t_state_unique);
    audit.nearest_distance.steering = nearest_sample_distance( ...
        t_uniform_absolute, t_steer_unique);
    audit.nearest_distance.steering_command = nearest_sample_distance( ...
        t_uniform_absolute, t_steerCmd_unique);
    audit.nearest_distance.torque_vectoring = nearest_sample_distance( ...
        t_uniform_absolute, t_tv_unique);
end
end

function [keepStart, keepEnd] = vx_moving_window(vx, Ts, vxThreshold, confirmTime)
%VX_MOVING_WINDOW Return indices for the main moving segment.
% Uses a short moving average + a required consecutive duration to avoid
% trimming based on noise spikes.

n = numel(vx);
if n == 0
    keepStart = 1;
    keepEnd = 0;
    return;
end

confirmSamples = max(1, ceil(confirmTime / Ts));

% Smooth abs(vx) over the same confirmation horizon
speed = abs(vx);
speedSmooth = movmean(speed, confirmSamples, 'Endpoints', 'shrink');

isMoving = speedSmooth > vxThreshold;

% Require a full run of confirmSamples consecutive moving samples.
% Use 'valid' windows to avoid edge effects.
if confirmSamples >= n
    keepStart = 1;
    keepEnd = n;
    return;
end

runOk = conv(double(isMoving), ones(confirmSamples, 1), 'valid') >= confirmSamples;
firstIdx = find(runOk, 1, 'first');
lastIdx  = find(runOk, 1, 'last');

if isempty(firstIdx) || isempty(lastIdx) || firstIdx >= lastIdx
    keepStart = 1;
    keepEnd = n;
    return;
end

% 'valid' output indices correspond to window start indices
keepStart = firstIdx;
keepEnd = lastIdx + confirmSamples - 1;
end

function assert_topic_not_empty(t, topic)
if isempty(t)
    error('Required topic "%s" contains no messages.', topic);
end
end

function assert_monotonic_time(t, topic)
if any(diff(t) < 0)
    error('Topic "%s" contains backward timestamp jumps.', topic);
end
end

function stats = timestamp_stats(t, Ts)
dt = diff(t(:));
positive_dt = dt(dt > 0);

stats.message_count = numel(t);
stats.duration = t(end) - t(1);
stats.duplicate_count = sum(dt == 0);
stats.backward_count = sum(dt < 0);
stats.gaps_over_2Ts = sum(dt > 2*Ts);
stats.gaps_over_3Ts = sum(dt > 3*Ts);

if isempty(positive_dt)
    stats.median_dt = NaN;
    stats.p95_dt = NaN;
    stats.max_dt = NaN;
    stats.median_rate_hz = NaN;
else
    stats.median_dt = median(positive_dt);
    stats.p95_dt = prctile(positive_dt, 95);
    stats.max_dt = max(positive_dt);
    stats.median_rate_hz = 1 / stats.median_dt;
end
end

function distance = nearest_sample_distance(query_time, raw_time)
nearest_time = interp1(raw_time, raw_time, query_time, 'nearest');
distance = abs(query_time - nearest_time);
end
