function x_next = sim_ltv(X, U, vx_next, dt)
% LTV-based simulator consistent with MPC model
%
% X = [x, y, psi, vx, vy, r]
% U = [delta, M_TV]

    % Extract states
    x   = X(1);
    y   = X(2);
    psi = X(3);
    vx  = X(4);
    vy  = X(5);
    r   = X(6);

    % Inputs
    U = U(:);
    delta = U(1);
    mz    = U(2);

    % Build local state
    x_local = [y; vy; psi; r];
    x_local = x_local(:);

    % Get discrete LTV model at this operating point
    [Ad, Bd, Cd] = ltv_tv_matrix(x_local, U, vx);

    % Propagate local dynamics
    x_local_next = Ad * x_local + Bd * U + Cd;

    % Extract updated states
    y_next   = x_local_next(1);
    vy_next  = x_local_next(2);
    psi_next = x_local_next(3);
    r_next   = x_local_next(4);

    % Global position update (same as your nonlinear sim)
    x_dot = vx*cos(psi) - vy*sin(psi);
    y_dot = vx*sin(psi) + vy*cos(psi);

    x_next_pos = x + x_dot * dt;
    y_next_pos = y + y_dot * dt;

    % Assemble full state
    x_next = [
        x_next_pos;
        y_next_pos;
        psi_next;
        vx_next;
        vy_next;
        r_next
    ];
end