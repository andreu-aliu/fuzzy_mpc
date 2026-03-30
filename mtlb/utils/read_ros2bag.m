function data = read_ros2bag(bagPath)

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
steerSel  = select(bag,"Topic","/as/c/steering");
steerMsgs = readMessages(steerSel);
t_steer = steerSel.MessageList.Time;

Nst = numel(steerMsgs);

steering = zeros(Nst,1);

for i = 1:Nst
    msg = steerMsgs{i};

    steering(i) = msg.steering;
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

    mz(i)  = msg.desired_mz;
    Tfl(i) = msg.front_left_torque;
    Tfr(i) = msg.front_right_torque;
    Trl(i) = msg.rear_left_torque;
    Trr(i) = msg.rear_right_torque;
end

% REMOVE DUPLICATES
t0 = t_state(1);

t_state = t_state - t0;
t_steer = t_steer - t0;
t_tv    = t_tv - t0;

[t_steer_unique, idx_steer] = unique(t_steer, 'stable');
steering_unique = steering(idx_steer);

[t_tv_unique, idx_tv] = unique(t_tv, 'stable');

mz_unique  = mz(idx_tv);
Tfl_unique = Tfl(idx_tv);
Tfr_unique = Tfr(idx_tv);
Trl_unique = Trl(idx_tv);
Trr_unique = Trr(idx_tv);

% RESAMPLING
Ts = 0.01;  
t_start = max([t_state(1), t_steer_unique(1), t_tv_unique(1)]);
t_end   = min([t_state(end), t_steer_unique(end), t_tv_unique(end)]);

t_uniform = (t_start:Ts:t_end)';

x_100  = interp1(t_state, x,  t_uniform, 'linear');
y_100  = interp1(t_state, y,  t_uniform, 'linear');
vx_100 = interp1(t_state, vx, t_uniform, 'linear');
vy_100 = interp1(t_state, vy, t_uniform, 'linear');
r_100  = interp1(t_state, r,  t_uniform, 'linear');

steering_100 = interp1(t_steer_unique, steering_unique, ...
                       t_uniform, 'linear');

mz_100  = interp1(t_tv_unique, mz_unique,  t_uniform, 'linear');
Tfl_100 = interp1(t_tv_unique, Tfl_unique, t_uniform, 'linear');
Tfr_100 = interp1(t_tv_unique, Tfr_unique, t_uniform, 'linear');
Trl_100 = interp1(t_tv_unique, Trl_unique, t_uniform, 'linear');
Trr_100 = interp1(t_tv_unique, Trr_unique, t_uniform, 'linear');


% CREATE TIMESERIES
data.time = t_uniform;

data.x  = x_100;
data.y  = y_100;
data.vx = vx_100;
data.vy = vy_100;
data.r  = r_100;

data.st = steering_100;

data.mz  = mz_100;
data.Tfl = Tfl_100;
data.Tfr = Tfr_100;
data.Trl = Trl_100;
data.Trr = Trr_100;
end