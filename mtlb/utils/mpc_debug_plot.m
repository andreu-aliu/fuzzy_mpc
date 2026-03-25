function mpc_debug_plot(k, x0, x_ref, x_pred, u_pred, params, opts)
%MPC_DEBUG_PLOT Visualize MPC states, references and controls
%
% opts fields:
%   .enabled   (true/false)
%   .step      (specific step to plot, [] = all)
%   .pause     (true/false)
%   .figure_id (optional)

    if ~opts.enabled
        return;
    end

    if ~isempty(opts.step) && k ~= opts.step
        return;
    end

    if ~isfield(opts,'figure_id')
        fig_id = 99;
    else
        fig_id = opts.figure_id;
    end

    % Persistent handles → reused every iteration
    persistent hFig hAx hLines initialized

    Np = size(x_ref,1);
    t_pred = 1:Np;

    if isempty(initialized) || ~isgraphics(hFig) || ~isgraphics(hLines.y_ref)

        hFig = figure(fig_id); clf;
        tl = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

        % Create axes
        hAx = gobjects(6,1);
        for i = 1:6
            hAx(i) = nexttile(tl);
            hold(hAx(i),'on'); grid(hAx(i),'on');
        end

        % Pre-create plot lines (FASTER updates)
        hLines.y_ref  = plot(hAx(1), t_pred, x_ref(:,1),'w--','LineWidth',1.5);
        hLines.y_pred = plot(hAx(1), t_pred, x_pred(:,1),'b','LineWidth',1.5);
        hLines.y_x0   = yline(hAx(1), x0(1),'r:','LineWidth',1.5);
        title(hAx(1),'y');

        hLines.vy_ref  = plot(hAx(2), t_pred, x_ref(:,2),'w--','LineWidth',1.5);
        hLines.vy_pred = plot(hAx(2), t_pred, x_pred(:,2),'b','LineWidth',1.5);
        hLines.vy_x0   = yline(hAx(2), x0(2),'r:','LineWidth',1.5);
        title(hAx(2),'v_y');

        hLines.psi_ref  = plot(hAx(3), t_pred, x_ref(:,3),'w--','LineWidth',1.5);
        hLines.psi_pred = plot(hAx(3), t_pred, x_pred(:,3),'b','LineWidth',1.5);
        hLines.psi_x0   = yline(hAx(3), x0(3),'r:','LineWidth',1.5);
        title(hAx(3),'\psi');

        hLines.r_ref  = plot(hAx(4), t_pred, x_ref(:,4),'w--','LineWidth',1.5);
        hLines.r_pred = plot(hAx(4), t_pred, x_pred(:,4),'b','LineWidth',1.5);
        hLines.r_x0   = yline(hAx(4), x0(4),'r:','LineWidth',1.5);
        title(hAx(4),'r');

        hLines.st = plot(hAx(5), t_pred, u_pred(:,1),'LineWidth',1.5);
        yline(hAx(5), params.min_st,'r--');
        yline(hAx(5), params.max_st,'r--');
        title(hAx(5),'\delta');

        hLines.mz = plot(hAx(6), t_pred, u_pred(:,2),'LineWidth',1.5);
        yline(hAx(6), params.min_mz,'r--');
        yline(hAx(6), params.max_mz,'r--');
        title(hAx(6),'M_z');

        sgtitle(sprintf('MPC Debug (k = %d)',k));

        initialized = true;

    else
        % Update data instead of replotting

        set(hLines.y_ref,  'YData', x_ref(:,1));
        set(hLines.y_pred, 'YData', x_pred(:,1));
        set(hLines.y_x0,   'Value', x0(1));

        set(hLines.vy_ref,  'YData', x_ref(:,2));
        set(hLines.vy_pred, 'YData', x_pred(:,2));
        set(hLines.vy_x0,   'Value', x0(2));

        set(hLines.psi_ref,  'YData', x_ref(:,3));
        set(hLines.psi_pred, 'YData', x_pred(:,3));
        set(hLines.psi_x0,   'Value', x0(3));

        set(hLines.r_ref,  'YData', x_ref(:,4));
        set(hLines.r_pred, 'YData', x_pred(:,4));
        set(hLines.r_x0,   'Value', x0(4));

        set(hLines.st, 'YData', u_pred(:,1));
        set(hLines.mz, 'YData', u_pred(:,2));

        sgtitle(sprintf('MPC Debug (k = %d)',k));
    end

    drawnow;

    if isfield(opts,'pause') && opts.pause
        pause;
    end
end