function data = read_ros2bag(bagPath)

% READ_ROS2_CAR_DATA
% Reads ROS2 bag and returns synchronized timeseries object
% All signals are interpolated to /as/c/state timestamps

% Open bag
bag = ros2bagreader(bagPath);

% ==========================
% STATE TOPIC (REFERENCE)
% ==========================

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

% ==========================
% STEERING
% ==========================

steerSel  = select(bag,"Topic","/as/c/steering");
steerMsgs = readMessages(steerSel);
t_steer = steerSel.MessageList.Time;

Nst = numel(steerMsgs);

steering = zeros(Nst,1);

for i = 1:Nst
    msg = steerMsgs{i};

    steering(i) = msg.steering;
end

% ==========================
% TORQUE VECTORING
% ==========================

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

% ==========================
% REMOVE DUPLICATES
% ==========================

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

% ==========================
% ZERO-ORDER HOLD (PREVIOUS SAMPLE)
% ==========================

steering_sync = interp1(t_steer_unique, steering_unique, ...
                        t_state, 'previous', 'extrap');

mz_sync  = interp1(t_tv_unique, mz_unique,  t_state, 'previous', 'extrap');
Tfl_sync = interp1(t_tv_unique, Tfl_unique, t_state, 'previous', 'extrap');
Tfr_sync = interp1(t_tv_unique, Tfr_unique, t_state, 'previous', 'extrap');
Trl_sync = interp1(t_tv_unique, Trl_unique, t_state, 'previous', 'extrap');
Trr_sync = interp1(t_tv_unique, Trr_unique, t_state, 'previous', 'extrap');

% ==========================
% CREATE TIMESERIES
% ==========================

data.time = t_state;

data.x  = x;
data.y  = y;
data.vx = vx;
data.vy = vy;
data.r  = r;

data.steering = steering_sync;

data.mz  = mz_sync;
data.Tfl = Tfl_sync;
data.Tfr = Tfr_sync;
data.Trl = Trl_sync;
data.Trr = Trr_sync;
end