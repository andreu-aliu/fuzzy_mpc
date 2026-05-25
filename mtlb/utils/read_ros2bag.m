function data = read_ros2bag(bagPath, Ts)

% READ_ROS2_CAR_DATA
% Reads ROS2 bag and returns synchronized timeseries object
% All signals are interpolated to /as/c/state timestamps

% Open bag
bag = ros2bagreader(bagPath);

% STATE TOPIC (REFERENCE)
stateSel  = select(bag,"Topic","/as/c/state");
stateMsgs = readMessages(stateSel);
t_state = stateSel.MessageList.Time;

Ns = numel(stateMsgs);

x  = zeros(Ns,1);
y  = zeros(Ns,1);
vx = zeros(Ns,1);
vy = zeros(Ns,1);
r  = zeros(Ns,1);

for i = 1:Ns
    msg = stateMsgs{i};

    x(i)  = msg.odom.position.x;
    y(i)  = msg.odom.position.y;
    vx(i) = msg.odom.velocity.x;
    vy(i) = msg.odom.velocity.y;
    r(i)  = msg.odom.velocity.w;
end

% STEERING
steerSel  = select(bag,"Topic","/el/sensor/driver_inputs");
steerMsgs = readMessages(steerSel);
t_steer = steerSel.MessageList.Time;

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

% REMOVE DUPLICATES
t0 = t_state(1);

t_state    = t_state - t0;
t_steer    = t_steer - t0;
t_steerCmd = t_steerCmd - t0;
t_tv       = t_tv - t0;

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

x_100  = interp1(t_state, x,  t_uniform, 'linear');
y_100  = interp1(t_state, y,  t_uniform, 'linear');
vx_100 = interp1(t_state, vx, t_uniform, 'linear');
vy_100 = interp1(t_state, vy, t_uniform, 'linear');
r_100  = interp1(t_state, r,  t_uniform, 'linear');

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

t_uniform = t_uniform(keepStart:keepEnd);
t_uniform = t_uniform - t_uniform(1);

x_100  = x_100(keepStart:keepEnd);
y_100  = y_100(keepStart:keepEnd);
vx_100 = vx_100(keepStart:keepEnd);
vy_100 = vy_100(keepStart:keepEnd);
r_100  = r_100(keepStart:keepEnd);

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
data.vx = vx_100;
data.vy = vy_100;
data.r  = r_100;

data.delta = steering_100;

data.st = steeringCmd_100;

data.mz  = mz_100;
data.Tfl = Tfl_100;
data.Tfr = Tfr_100;
data.Trl = Trl_100;
data.Trr = Trr_100;
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
