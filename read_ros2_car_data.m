function data = read_ros2_car_data(bagPath)

% READ_ROS2_CAR_DATA
% Reads selected signals from a ROS2 bag (MCAP or sqlite3)
%
% Required topics:
%   /as/c/state
%   /as/c/steering
%   /ctrl/llc/torque_vectoring
%
% Returns struct "data" with time vectors and signals.

% Open bag
bag = ros2bagreader(bagPath);

% -------------------------------
%  STATE TOPIC
% -------------------------------

stateSel = select(bag,"Topic","/as/c/state");
stateMsgs = readMessages(stateSel);

Ns = numel(stateMsgs);

t_state = zeros(Ns,1);
x       = zeros(Ns,1);
y       = zeros(Ns,1);
vx      = zeros(Ns,1);
vy      = zeros(Ns,1);

for i = 1:Ns
    msg = stateMsgs{i};

    t_state(i) = double(msg.header.stamp.sec) + ...
                 double(msg.header.stamp.nanosec)*1e-9;

    x(i)  = msg.odom.position.x;
    y(i)  = msg.odom.position.y;
    vx(i) = msg.odom.velocity.x;
    vy(i) = msg.odom.velocity.y;
end

t_state = t_state - t_state(1);

% -------------------------------
%  STEERING TOPIC
% -------------------------------

steerSel = select(bag,"Topic","/as/c/steering");
steerMsgs = readMessages(steerSel);

Nst = numel(steerMsgs);

t_steer  = zeros(Nst,1);
steering = zeros(Nst,1);

for i = 1:Nst
    msg = steerMsgs{i};

    t_steer(i) = double(msg.header.stamp.sec) + ...
                 double(msg.header.stamp.nanosec)*1e-9;

    steering(i) = msg.steering;
end

t_steer = t_steer - t_steer(1);

% -------------------------------
%  TORQUE VECTORING TOPIC
% -------------------------------

tvSel = select(bag,"Topic","/ctrl/llc/torque_vectoring");
tvMsgs = readMessages(tvSel);

Nt = numel(tvMsgs);

t_tv = zeros(Nt,1);

actual_mz = zeros(Nt,1);
T_fl = zeros(Nt,1);
T_fr = zeros(Nt,1);
T_rl = zeros(Nt,1);
T_rr = zeros(Nt,1);

for i = 1:Nt
    msg = tvMsgs{i};

    t_tv(i) = double(msg.header.stamp.sec) + ...
              double(msg.header.stamp.nanosec)*1e-9;

    actual_mz = msg.actual_mz;

    T_fl(i) = msg.front_left_torque;
    T_fr(i) = msg.front_right_torque;
    T_rl(i) = msg.rear_left_torque;
    T_rr(i) = msg.rear_right_torque;
end

t_tv = t_tv - t_tv(1);

% -------------------------------
%  Pack output
% -------------------------------

data.state.time = t_state;
data.state.x    = x;
data.state.y    = y;
data.state.vx   = vx;
data.state.vy   = vy;

data.steering.time     = t_steer;
data.steering.steering = steering;

data.torque.time  = t_tv;
data.torque.actual_mz = actual_mz;
data.torque.front_left  = T_fl;
data.torque.front_right = T_fr;
data.torque.rear_left   = T_rl;
data.torque.rear_right  = T_rr;

end