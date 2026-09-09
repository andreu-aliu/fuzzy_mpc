function eefig_model_insights(learner, history, evaluation, prepared)
%EEFIG_MODEL_INSIGHTS Plot structure evolution and frozen evaluation quality.

fprintf('\nEEFIG structure summary:\n');
granule_id = (1:learner.NG)';
admitted_samples = cellfun(@(g) g.n_samples, learner.granules(:));
membership_mass = learner.membershipMass(:);
performance_index = learner.performanceIndex(:);
activation_count = zeros(learner.NG, 1);
centers = zeros(learner.NG, 4);
covariance_condition = zeros(learner.NG, 1);
physical_scale = [prepared.scale_x; prepared.scale_u];

all_active = vertcat(evaluation.runs.active_granule);
for granule_idx = 1:learner.NG
    activation_count(granule_idx) = sum(all_active == granule_idx);
    centers(granule_idx, :) = ...
        (learner.granules{granule_idx}.nu .* physical_scale)';
    covariance_condition(granule_idx) = ...
        cond(learner.granules{granule_idx}.Sigma);
end

granule_table = table(granule_id, admitted_samples, membership_mass, ...
    performance_index, activation_count, centers(:, 1), centers(:, 2), ...
    centers(:, 3), centers(:, 4), covariance_condition, ...
    'VariableNames', {'Granule','AdmittedSamples','MembershipMass', ...
    'PJGIndex','EvaluationActivations','Center_vy','Center_r', ...
    'Center_vx','Center_delta','CovarianceCondition'});
disp(granule_table);

figure('Name', 'EEFIG training evolution');
layout = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
stairs(history.sample, history.granules, 'LineWidth', 1.4);
ylabel('Granules');
title('EEFIG structure evolution');
grid on;

nexttile;
plot(history.sample, cumsum(history.anomaly), 'LineWidth', 1.3, ...
    'DisplayName', 'Anomalies');
hold on;
plot(history.sample, cumsum(history.created), 'LineWidth', 1.3, ...
    'DisplayName', 'Granules created');
plot(history.sample, cumsum(history.reverted_updates), 'LineWidth', 1.3, ...
    'DisplayName', 'PJG reversions');
hold off;
ylabel('Cumulative count');
legend('Location', 'best');
grid on;

nexttile;
plot(history.sample, history.active_granule, '.', 'MarkerSize', 4);
xlabel('Training transition');
ylabel('Active granule');
grid on;
title(layout, sprintf('%d training runs, %d samples, %d final granules', ...
    numel(prepared.training_runs), numel(history.sample), learner.NG));

measured = [evaluation.runs.measured];
predicted = [evaluation.runs.predicted];
errors = predicted - measured;

figure('Name', 'EEFIG frozen one-step evaluation');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile;
scatter(measured(1, :), predicted(1, :), 5, '.', ...
    'MarkerEdgeAlpha', 0.15);
hold on;
plotIdentity(measured(1, :));
hold off;
xlabel('Measured v_y(k+1) [m/s]');
ylabel('Predicted v_y(k+1) [m/s]');
title('Lateral velocity');
axis equal;
grid on;

nexttile;
scatter(measured(2, :), predicted(2, :), 5, '.', ...
    'MarkerEdgeAlpha', 0.15);
hold on;
plotIdentity(measured(2, :));
hold off;
xlabel('Measured r(k+1) [rad/s]');
ylabel('Predicted r(k+1) [rad/s]');
title('Yaw rate');
axis equal;
grid on;

nexttile;
histogram(errors(1, :), 80);
xlabel('v_y prediction error [m/s]');
ylabel('Samples');
grid on;

nexttile;
histogram(errors(2, :), 80);
xlabel('r prediction error [rad/s]');
ylabel('Samples');
grid on;

figure('Name', 'EEFIG evaluation granule usage');
bar(granule_id, activation_count);
xlabel('Granule');
ylabel('Frozen evaluation activations');
title('Winning granule on held-out samples');
grid on;

plotGranuleEllipsoids4D(learner, physical_scale);
end

function plotIdentity(values)
limits = [min(values), max(values)];
if limits(1) == limits(2)
    limits = limits + [-1, 1] * max(abs(limits(1)), 1) * 1e-3;
end
plot(limits, limits, 'k--', 'LineWidth', 1.0);
end

function plotGranuleEllipsoids4D(learner, physical_scale)
%PLOTGRANULEELLIPSOIDS4D Visualize the 4-D admission ellipsoids.
% The [vy r vx] projection is drawn in 3-D. Surface colour is the delta
% coordinate on the boundary of the original 4-D ellipsoid. For a point
% on the boundary of the projected ellipsoid, this fourth coordinate is
% the conditional centre implied by the full covariance, so correlations
% with steering are retained rather than discarded.

figure('Name', 'EEFIG four-dimensional granules');
axes_handle = axes();
hold(axes_handle, 'on');

[sphere_x, sphere_y, sphere_z] = sphere(24);
unit_sphere = [sphere_x(:)'; sphere_y(:)'; sphere_z(:)'];
D = diag(physical_scale);
center_handles = gobjects(learner.NG, 1);

for granule_idx = 1:learner.NG
    granule = learner.granules{granule_idx};
    center = granule.nu .* physical_scale;
    covariance = D * granule.Sigma * D;
    covariance = 0.5 * (covariance + covariance');

    % Projection of the four-dimensional Mahalanobis ellipsoid onto the
    % first three physical coordinates.
    covariance_xyz = covariance(1:3, 1:3);
    covariance_xyz = 0.5 * (covariance_xyz + covariance_xyz');
    [V, eigenvalue_matrix] = eig(covariance_xyz);
    eigenvalues = max(real(diag(eigenvalue_matrix)), learner.regularization);
    transform = real(V * diag(sqrt(learner.epsilon * eigenvalues)));
    points = center(1:3) + transform * unit_sphere;

    % On the projected boundary, delta is uniquely given by the
    % conditional centre of the fourth dimension.
    delta_gain = covariance(4, 1:3) * pinv(covariance_xyz);
    delta_surface = center(4) + delta_gain * (points - center(1:3));

    surf(axes_handle, reshape(points(1, :), size(sphere_x)), ...
        reshape(points(2, :), size(sphere_y)), ...
        reshape(points(3, :), size(sphere_z)), ...
        reshape(delta_surface, size(sphere_x)), ...
        'EdgeColor', 'none', 'FaceAlpha', 0.22, ...
        'HandleVisibility', 'off');

    center_handles(granule_idx) = scatter3(axes_handle, center(1), ...
        center(2), center(3), 55, center(4), 'filled', ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', sprintf('Granule %d', granule_idx));
    text(axes_handle, center(1), center(2), center(3), ...
        sprintf('  %d', granule_idx), 'FontWeight', 'bold');
end

hold(axes_handle, 'off');
xlabel(axes_handle, 'v_y [m/s]');
ylabel(axes_handle, 'r [rad/s]');
zlabel(axes_handle, 'v_x [m/s]');
title(axes_handle, ...
    'Projection of 4-D admission ellipsoids (colour: steering angle)');
grid(axes_handle, 'on');
view(axes_handle, 3);
colormap(axes_handle, turbo);
colour_bar = colorbar(axes_handle);
colour_bar.Label.String = '\delta [rad]';
legend(axes_handle, center_handles, 'Location', 'eastoutside');
end
