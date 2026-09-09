classdef EEFIGLearning < handle
    %EEFIGLEARNING Online/offline learner for a TS-EEFIG fuzzy model.
    %
    % This class is the manager of a set of TSGranule objects G_k^i.
    % It is intentionally structured around the same blocks as the old
    % learn_EEFIG/data_evaluation code and Algorithm 1 of the paper:
    %
    %   1) Build/evaluate the premise vector zeta_k = [x_k; u_k].
    %   2) Compute memberships before antecedent updates.
    %   3) Update the antecedent of every sufficiently active granule.
    %   4) Recompute memberships after antecedent updates.
    %   5) Detect anomalies.
    %   6) Update the auxiliary tracker and test c-separation.
    %   7) Create a new granule if persistent anomalies are separated.
    %   8) Update consequents A_k^i, B_k^i using RLS online or WLS offline.
    %
    % Required TSGranule interface:
    %   granule.membership(zeta)
    %   granule.contains(zeta, epsilon)
    %   granule.mahalanobis(zeta)
    %   granule.updateAntecedent(zeta, g_i)
    %   granule.updateConsequentRLS(zeta, x_next)
    %   granule.initConsequentWLS(Zeta_window, Xnext_window)
    %   granule.predictLocal(x,u)
    %
    % Recommended usage for online learning:
    %   learner.updateOnline(x_k, u_k, x_k_plus_1);
    %
    % Recommended usage for offline learning:
    %   learner.trainOffline(X, U);
    % where X is nx x N and U is nu x (N-1) or nu x N.

    properties
        % Problem dimensions
        nx                      % Number of system states n_x
        nu_in                   % Number of control inputs n_u
        nzeta                   % Dimension of premise vector zeta = [x; u]

        % Granule set
        granules                % Cell array of TSGranule objects G_k^i
        NG                      % Current number of granules N_G
        membershipMass          % Running sum of normalized memberships used by the PJG index
        performanceIndex        % Accumulated PJG performance index per granule

        % Ellipsoidal admission and granule update parameters
        epsilon                 % Mahalanobis threshold defining E_k^i = {zeta | M_k^i <= epsilon}
        confidence              % Optional chi-square confidence used to compute epsilon
        membership_update_threshold % Old-code style threshold; granules with g_i below this are not tested for update
        use_pjg_quality_check   % If true, revert antecedent updates that reduce the PJG index

        % Memory and anomaly parameters
        phi                     % Memory horizon phi: number of recent samples kept in sliding windows
        n_anomaly_max           % Maximum allowed consecutive anomalies before creating a new granule
        anomaly_counter         % Current number of consecutive anomalies n_a
        anomalyZetaBuffer       % Buffer of anomalous premise vectors zeta_k
        anomalyXNextBuffer      % Buffer of targets x_{k+1} associated with anomalous premise vectors
        clear_anomaly_buffer_on_normal % If true, clear anomaly buffers after a normal sample

        % Auxiliary tracker G_aux used for c-separation
        tracker_nu              % Centre of auxiliary data tracker nu_aux
        tracker_C               % Inverse covariance of auxiliary tracker C_aux = Sigma_aux^{-1}
        tracker_initialized     % True once the tracker has received a sample
        tracker_forgetting      % Deprecated compatibility parameter (tracker uses the phi-window)
        tracker_effective_N     % Deprecated compatibility parameter (tracker uses the phi-window)
        c_separation            % c coefficient in the c-separation condition
        use_c_separation        % If true, require c-separation before creating a new granule

        % Consequent learning parameters
        rls_forgetting          % RLS forgetting factor eta/ff
        rls_mode                % 'global' follows the paper; 'per_granule' is retained for compatibility
        global_P                % Shared RLS inverse autocorrelation matrix P_k
        global_K                % Last shared RLS gain Upsilon_k

        % Sliding regression buffers
        zetaWindow              % nzeta x N sliding window of premise vectors zeta_k
        xNextWindow             % nx x N sliding window of targets x_{k+1}
        min_initial_samples     % Number of samples required before creating the first granule

        % General parameters
        granule_params          % Struct passed to each TSGranule
        regularization          % Small positive value used for numerical regularization
        k                       % Number of processed training pairs
        debug                   % If true, print basic update information
    end

    methods
        function obj = EEFIGLearning(nx, nu_in, params)
            %EEFIGLEARNING Construct an empty TS-EEFIG learner.
            %
            % Inputs:
            %   nx      - number of states
            %   nu_in   - number of control inputs
            %   params  - struct containing learner and granule parameters

            if nargin < 3
                params = struct();
            end

            obj.nx = nx;
            obj.nu_in = nu_in;
            obj.nzeta = nx + nu_in;

            obj.granules = {};
            obj.NG = 0;
            obj.membershipMass = zeros(0,1);
            obj.performanceIndex = zeros(0,1);

            obj.confidence = getFieldOrDefault(params, 'confidence', 0.95);
            obj.epsilon = getFieldOrDefault(params, 'epsilon', chi2inv(obj.confidence, obj.nzeta));
            obj.membership_update_threshold = getFieldOrDefault(params, 'membership_update_threshold', 1e-6);
            obj.use_pjg_quality_check = getFieldOrDefault(params, 'use_pjg_quality_check', true);

            obj.phi = getFieldOrDefault(params, 'phi', 30);
            obj.n_anomaly_max = getFieldOrDefault(params, 'n_anomaly_max', 5);
            obj.anomaly_counter = 0;
            obj.anomalyZetaBuffer = zeros(obj.nzeta, 0);
            obj.anomalyXNextBuffer = zeros(obj.nx, 0);
            obj.clear_anomaly_buffer_on_normal = getFieldOrDefault(params, 'clear_anomaly_buffer_on_normal', true);

            obj.tracker_nu = zeros(obj.nzeta, 1);
            obj.tracker_C = eye(obj.nzeta);
            obj.tracker_initialized = false;
            obj.tracker_forgetting = getFieldOrDefault(params, 'tracker_forgetting', 0.99);
            obj.tracker_effective_N = getFieldOrDefault(params, 'tracker_effective_N', 200);
            obj.c_separation = getFieldOrDefault(params, 'c_separation', 2.0);
            obj.use_c_separation = getFieldOrDefault(params, 'use_c_separation', true);

            obj.rls_forgetting = getFieldOrDefault(params, 'rls_forgetting', 0.99);
            obj.rls_mode = getFieldOrDefault(params, 'rls_mode', 'global');
            obj.global_P = getFieldOrDefault(params, 'global_P', 1e5 * eye(obj.nzeta));
            obj.global_K = zeros(obj.nzeta, 1);

            obj.zetaWindow = zeros(obj.nzeta, 0);
            obj.xNextWindow = zeros(obj.nx, 0);
            obj.min_initial_samples = getFieldOrDefault(params, 'min_initial_samples', max(2, obj.phi));

            obj.granule_params = params;
            obj.granule_params.eta = getFieldOrDefault(params, 'eta', obj.rls_forgetting);
            obj.regularization = getFieldOrDefault(params, 'reg', 1e-8);

            obj.k = 0;
            obj.debug = getFieldOrDefault(params, 'debug', false);
        end

        function status = updateOnline(obj, x, u, x_next)
            %UPDATEONLINE Online learning update from one transition.
            %
            % Inputs:
            %   x       - current state x_k
            %   u       - current input u_k
            %   x_next  - next state x_{k+1}
            %
            % The premise vector is zeta_k = [x_k; u_k].
            % Antecedents are updated from zeta_k.
            % Consequents are updated from the regression pair:
            %   zeta_k -> x_{k+1}.

            zeta = [x(:); u(:)];
            status = obj.updatePair(zeta, x_next(:), 'online');
        end

        function status = updatePair(obj, zeta, x_next, mode)
            %UPDATEPAIR Core update from one regression pair.
            %
            % Inputs:
            %   zeta    - premise vector zeta_k = [x_k; u_k]
            %   x_next  - target state x_{k+1}
            %   mode    - 'online' or 'offline'
            %
            % Behaviour:
            %   - In 'online' mode, the active consequent is updated with RLS.
            %   - In 'offline' mode, the active consequent is fitted by WLS
            %     from the current moving window; no RLS is applied.

            if nargin < 4
                mode = 'online';
            end

            zeta = zeta(:);
            x_next = x_next(:);

            obj.k = obj.k + 1;
            obj.appendRegressionSample(zeta, x_next);
            obj.updateAuxTracker(zeta);

            status = obj.emptyStatus();
            status.mode = mode;

            if obj.NG == 0
                status.buffering = true;
                if size(obj.zetaWindow, 2) >= obj.min_initial_samples
                    obj.createFirstGranuleFromWindow();
                    status.buffering = false;
                    status.created_first_granule = true;
                    status.created_new_granule = true;
                    status.active_idx = obj.NG;
                end
                return;
            end

            % First pass: compute memberships before changing any granule.
            [g_pre, xi_pre] = obj.computeMemberships(zeta);
            status.g_pre = g_pre;
            status.xi_pre = xi_pre;

            % Update antecedents of all granules that are sufficiently active.
            [is_anomaly, accepted_idx, reverted_idx] = obj.evaluateAndUpdateAntecedents(zeta, g_pre);
            status.is_anomaly = is_anomaly;
            status.accepted_idx = accepted_idx;
            status.reverted_idx = reverted_idx;

            % Second pass: recompute memberships after antecedent updates.
            [g_post, xi_post] = obj.computeMemberships(zeta);
            [~, active_idx] = max(g_post);
            status.g_post = g_post;
            status.xi_post = xi_post;
            status.active_idx = active_idx;

            % Anomaly bookkeeping and possible new granule creation.
            if is_anomaly
                obj.anomaly_counter = obj.anomaly_counter + 1;
                obj.appendAnomalySample(zeta, x_next);
            else
                obj.anomaly_counter = 0;
                if obj.clear_anomaly_buffer_on_normal
                    obj.clearAnomalyBuffers();
                end
            end
            status.anomaly_counter = obj.anomaly_counter;

            created_new_granule = false;
            if obj.shouldCreateNewGranule()
                obj.createGranuleFromAnomaliesOrWindow();
                created_new_granule = true;
                status.created_new_granule = true;
                status.active_idx = obj.NG;
                obj.anomaly_counter = 0;
                obj.clearAnomalyBuffers();

                % Recompute memberships after the granule set changes.
                [g_post, xi_post] = obj.computeMemberships(zeta);
                [~, status.active_idx] = max(g_post);
                status.g_post = g_post;
                status.xi_post = xi_post;
            end

            % Consequent update. Algorithm 1 makes granule creation/WLS and
            % the ordinary consequent update mutually exclusive.
            if strcmpi(mode, 'online')
                if ~created_new_granule
                    obj.updateConsequentOnline(status.active_idx, zeta, x_next);
                end
            elseif strcmpi(mode, 'offline')
                if ~created_new_granule
                    % Equation (24): batch windowed least squares on the
                    % moving phi-sample window for the active granule.
                    obj.fitConsequentWLS(status.active_idx, obj.zetaWindow, obj.xNextWindow);
                end
            else
                error('Unknown learning mode: %s. Use online or offline.', mode);
            end

            if obj.debug
                fprintf('k=%d NG=%d anomaly=%d active=%d created=%d\n', ...
                    obj.k, obj.NG, status.is_anomaly, status.active_idx, status.created_new_granule);
            end
        end

        function trainOffline(obj, X, U)
            %TRAINOFFLINE Offline training from state and input trajectories.
            %
            % Inputs:
            %   X : nx x N state trajectory
            %   U : nu x (N-1) or nu x N input trajectory
            %
            % The training pairs are:
            %   zeta_k = [x_k; u_k], target = x_{k+1}
            % for k = 1,...,N-1.
            %
            % This function updates the structure sample by sample and uses
            % moving-window WLS for the active consequent at every step.

            N = size(X, 2);
            if size(U, 2) < N - 1
                error('U must have at least N-1 columns.');
            end

            Zeta = zeros(obj.nzeta, N-1);
            Xnext = zeros(obj.nx, N-1);

            for kidx = 1:N-1
                Zeta(:, kidx) = [X(:, kidx); U(:, kidx)];
                Xnext(:, kidx) = X(:, kidx + 1);
                obj.updatePair(Zeta(:, kidx), Xnext(:, kidx), 'offline');
            end

        end

        function trainOfflinePairs(obj, Zeta, Xnext)
            %TRAINOFFLINEPAIRS Offline training from explicit regression pairs.
            %
            % Inputs:
            %   Zeta  : nzeta x N matrix of premise vectors
            %   Xnext : nx x N matrix of target next states

            N = size(Zeta, 2);
            if size(Xnext, 2) ~= N
                error('Zeta and Xnext must contain the same number of samples.');
            end

            for kidx = 1:N
                obj.updatePair(Zeta(:, kidx), Xnext(:, kidx), 'offline');
            end

        end

        function [Abar, Bbar, g] = instantiateAB(obj, zeta)
            %INSTANTIATEAB Compute the global TS matrices Abar(zeta), Bbar(zeta).
            %
            % Abar(zeta) = sum_i g_i(zeta) A_i
            % Bbar(zeta) = sum_i g_i(zeta) B_i

            if obj.NG == 0
                error('Cannot instantiate Abar/Bbar: the model has no granules.');
            end

            [g, ~] = obj.computeMemberships(zeta(:));

            Abar = zeros(obj.nx, obj.nx);
            Bbar = zeros(obj.nx, obj.nu_in);

            for i = 1:obj.NG
                Abar = Abar + g(i) * obj.granules{i}.A;
                Bbar = Bbar + g(i) * obj.granules{i}.B;
            end
        end

        function x_next_pred = predict(obj, x, u)
            %PREDICT Predict x_{k+1} using the blended TS model.

            zeta = [x(:); u(:)];
            [Abar, Bbar, ~] = obj.instantiateAB(zeta);
            x_next_pred = Abar * x(:) + Bbar * u(:);
        end

        function [g, xi] = computeMemberships(obj, zeta)
            %COMPUTEMEMBERSHIPS Compute raw and normalized memberships.
            %
            % xi_i is the non-normalized membership of granule i.
            % g_i = xi_i / sum_j xi_j.
            %
            % This reproduces the old data_evaluation behaviour: compute all
            % raw weights, normalize them, and guard against all-zero weights.

            if obj.NG == 0
                g = zeros(0,1);
                xi = zeros(0,1);
                return;
            end

            zeta = zeta(:);
            xi = zeros(obj.NG, 1);

            for i = 1:obj.NG
                xi(i) = obj.granules{i}.membership(zeta);
            end

            xi_sum = sum(xi);
            if xi_sum <= obj.regularization || all(~isfinite(xi))
                g = ones(obj.NG, 1) / obj.NG;
            else
                g = xi / xi_sum;
            end
        end

        function [g, xi, lambda0, lambda1, f] = membershipGeometry(obj, zeta)
            %MEMBERSHIPGEOMETRY Membership and bound sensitivities for PJG.

            zeta = zeta(:);
            [g, xi] = obj.computeMemberships(zeta);
            lambda0 = zeros(obj.NG, obj.nzeta);
            lambda1 = zeros(obj.NG, obj.nzeta);
            f = zeros(obj.NG, 1);

            for q = 1:obj.NG
                gran = obj.granules{q};
                width = max(gran.zeta_max - gran.zeta_min, gran.reg);
                distance = zeta - gran.nu;
                f(q) = sqrt(sum((distance ./ width).^2));
                lambda0(q, :) = (2 * distance ./ width.^2)';
                lambda1(q, :) = (2 * distance.^2 ./ width.^3)';
            end
        end

        function [is_anomaly, accepted_idx, reverted_idx] = evaluateAndUpdateAntecedents(obj, zeta, g_pre)
            %EVALUATEANDUPDATEANTECEDENTS Update all admissible granule antecedents.
            %
            % This is the structured replacement of data_evaluation():
            %   1) loop over all granules,
            %   2) skip granules with negligible normalized membership,
            %   3) check ellipsoidal admission zeta in E_k^i,
            %   4) update nu, Sigma, zeta_min, zeta_max,
            %   5) revert if the PJG quality index worsens,
            %   6) mark the sample as non-anomalous if any granule accepts it.

            is_anomaly = true;
            accepted_idx = [];
            reverted_idx = [];

            for i = 1:obj.NG
                if isnan(g_pre(i)) || g_pre(i) < obj.membership_update_threshold
                    continue;
                end

                [accepted, reverted] = obj.tryUpdateOneGranuleAntecedent(i, zeta, g_pre(i));

                if accepted
                    is_anomaly = false;
                    accepted_idx(end+1) = i; %#ok<AGROW>
                end
                if reverted
                    reverted_idx(end+1) = i; %#ok<AGROW>
                end
            end
        end

        function [accepted, reverted] = tryUpdateOneGranuleAntecedent(obj, i, zeta, g_i)
            %TRYUPDATEONEGRANULEANTECEDENT Try to update granule i.
            %
            % The sample is admitted if zeta belongs to E_k^i. Its tentative
            % update is retained only when the equation-(14) index does not
            % decrease. Reverting an update does not turn an admitted sample
            % into an anomaly.

            accepted = false;
            reverted = false;
            gran = obj.granules{i};

            if ~gran.contains(zeta, obj.epsilon)
                return;
            end

            accepted = true;
            snapshot = obj.captureAntecedent(gran);
            [g_before, xi_before, lambda0, lambda1, f] = obj.membershipGeometry(zeta);
            oldQuality = obj.performanceIndex(i);

            gran.updateAntecedent(zeta, g_i);

            delta_lower = gran.zeta_min - snapshot.zeta_min;
            delta_center = gran.nu - snapshot.nu;
            delta_upper = gran.zeta_max - snapshot.zeta_max;
            candidateMass = obj.pjgMembershipMass(i, g_before, xi_before, ...
                lambda0, lambda1, f, delta_lower, delta_center, delta_upper);
            qualityIncrement = gran.mahalanobis(zeta) * candidateMass;
            newQuality = oldQuality + qualityIncrement;

            quality_tolerance = 100 * obj.regularization * max(1, abs(oldQuality));
            invalidCandidate = ~isfinite(candidateMass) || candidateMass <= 0 || ~isfinite(newQuality);
            if obj.use_pjg_quality_check && (invalidCandidate || newQuality + quality_tolerance < oldQuality)
                obj.restoreAntecedent(gran, snapshot);
                reverted = true;
            else
                if invalidCandidate
                    [g_candidate, ~] = obj.computeMemberships(zeta);
                    candidateMass = obj.membershipMass(i) + g_candidate(i);
                    newQuality = oldQuality + gran.mahalanobis(zeta) * candidateMass;
                end
                obj.membershipMass(i) = candidateMass;
                obj.performanceIndex(i) = newQuality;
            end
        end

        function candidateMass = pjgMembershipMass(obj, i, g, xi, lambda0, lambda1, f, delta_lower, delta_center, delta_upper)
            %PJGMEMBERSHIPMASS First-order PJG coverage update from [39].
            %
            % The bound and centre changes alter the normalized membership
            % of every rule. This sensitivity correction is the part that a
            % simple "old mass + g_i" approximation misses.

            sum_xi = max(sum(xi), obj.regularization);
            safe_f = max(f, sqrt(obj.regularization));
            weighted_l0 = sum((xi ./ safe_f) .* lambda0, 1);
            weighted_l1 = sum((xi ./ safe_f) .* lambda1, 1);

            d_g_d_lower = xi(i) * weighted_l1 / sum_xi^2 ...
                - xi(i) * lambda1(i, :) / (safe_f(i) * sum_xi);
            d_g_d_center = xi(i) * lambda0(i, :) / (safe_f(i) * sum_xi) ...
                - xi(i) * weighted_l0 / sum_xi^2;
            d_g_d_upper = xi(i) * lambda1(i, :) / (safe_f(i) * sum_xi) ...
                - xi(i) * weighted_l1 / sum_xi^2;

            correction = g(i) ...
                - d_g_d_lower * delta_lower ...
                + d_g_d_center * delta_center ...
                + d_g_d_upper * delta_upper;
            candidateMass = obj.membershipMass(i) + correction;
        end

        function updateConsequentOnline(obj, active_idx, zeta, x_next)
            %UPDATECONSEQUENTONLINE Update the active consequent using RLS.
            %
            % Only i* = argmax_i g_i is updated, following the policy in the
            % paper and the old learn_EEFIG implementation.

            if active_idx < 1 || active_idx > obj.NG
                return;
            end

            gran = obj.granules{active_idx};

            if isempty(gran.A) || isempty(gran.B)
                obj.fitConsequentWLS(active_idx, obj.zetaWindow, obj.xNextWindow);
                return;
            end

            if strcmpi(obj.rls_mode, 'per_granule')
                gran.updateConsequentRLS(zeta, x_next);
            elseif strcmpi(obj.rls_mode, 'global')
                obj.updateConsequentRLSGlobalP(active_idx, zeta, x_next);
            else
                error('Unknown rls_mode: %s.', obj.rls_mode);
            end
        end

        function updateConsequentRLSGlobalP(obj, active_idx, zeta, x_next)
            %UPDATECONSEQUENTRLSGLOBALP Equations (21)-(23), with shared P.
            %
            % The paper denotes a single P_k and Upsilon_k, shared across the
            % currently active local consequent.

            zeta = zeta(:);
            x_next = x_next(:);
            gran = obj.granules{active_idx};

            K = (obj.global_P * zeta) / (obj.rls_forgetting + zeta' * obj.global_P * zeta);
            obj.global_P = (eye(obj.nzeta) - K * zeta') * obj.global_P / obj.rls_forgetting;
            obj.global_P = 0.5 * (obj.global_P + obj.global_P');
            obj.global_K = K;

            theta = [gran.A, gran.B]';        % nzeta x nx
            residual = x_next' - zeta' * theta; % 1 x nx

            for j = 1:obj.nx
                theta(:, j) = theta(:, j) + K * residual(j);
            end

            Theta = theta';                   % nx x nzeta
            gran.Theta = Theta;
            gran.A = Theta(:, 1:obj.nx);
            gran.B = Theta(:, obj.nx+1:end);
        end

        function ensureConsequentExists(obj, idx)
            %ENSURECONSEQUENTEXISTS Fit WLS if a granule has no A/B yet.

            if idx < 1 || idx > obj.NG
                return;
            end
            if isempty(obj.granules{idx}.A) || isempty(obj.granules{idx}.B)
                obj.fitConsequentWLS(idx, obj.zetaWindow, obj.xNextWindow);
            end
        end

        function fitConsequentWLS(obj, idx, Zeta, Xnext)
            %FITCONSEQUENTWLS Fit A_i,B_i with batch least squares.
            %
            % Solves:
            %   Xnext ~= Theta_i * Zeta
            % where Theta_i = [A_i B_i].

            if size(Zeta, 2) < 1
                return;
            end

            gran = obj.granules{idx};
            gran.initConsequentWLS(Zeta, Xnext);
        end

        function fitAllConsequentsWLS(obj, Zeta, Xnext)
            %FITALLCONSEQUENTSWLS Optional post-fit, not Algorithm-1 training.
            %
            % Each sample is assigned to the granule with maximum current
            % membership. Then each local model is fitted using the samples
            % assigned to that granule. If a granule receives too few samples,
            % the full dataset is used as a fallback to keep A/B defined. The
            % standard trainOffline methods intentionally do not call this:
            % they use equation-(24) moving-window WLS sequentially.

            if obj.NG == 0
                error('Cannot fit consequents: the model has no granules.');
            end

            N = size(Zeta, 2);
            assignments = zeros(1, N);

            for kidx = 1:N
                [g, ~] = obj.computeMemberships(Zeta(:, kidx));
                [~, assignments(kidx)] = max(g);
            end

            for i = 1:obj.NG
                idx = find(assignments == i);
                if numel(idx) >= max(2, obj.nzeta)
                    obj.fitConsequentWLS(i, Zeta(:, idx), Xnext(:, idx));
                else
                    obj.fitConsequentWLS(i, Zeta, Xnext);
                end
            end
        end

        function appendRegressionSample(obj, zeta, x_next)
            %APPENDREGRESSIONSAMPLE Append one sample to the phi-window.

            obj.zetaWindow = [obj.zetaWindow, zeta(:)];
            obj.xNextWindow = [obj.xNextWindow, x_next(:)];

            if size(obj.zetaWindow, 2) > obj.phi
                obj.zetaWindow(:, 1) = [];
                obj.xNextWindow(:, 1) = [];
            end
        end

        function appendAnomalySample(obj, zeta, x_next)
            %APPENDANOMALYSAMPLE Store an anomalous sample.

            obj.anomalyZetaBuffer = [obj.anomalyZetaBuffer, zeta(:)];
            obj.anomalyXNextBuffer = [obj.anomalyXNextBuffer, x_next(:)];

            maxStored = max(obj.phi, obj.n_anomaly_max + 1);
            if size(obj.anomalyZetaBuffer, 2) > maxStored
                obj.anomalyZetaBuffer(:, 1) = [];
                obj.anomalyXNextBuffer(:, 1) = [];
            end
        end

        function clearAnomalyBuffers(obj)
            %CLEARANOMALYBUFFERS Clear stored anomalies.

            obj.anomalyZetaBuffer = zeros(obj.nzeta, 0);
            obj.anomalyXNextBuffer = zeros(obj.nx, 0);
        end

        function updateAuxTracker(obj, ~)
            %UPDATEAUXTRACKER Build G_aux from the current phi-sample memory.
            %
            % Section 2.2 defines the auxiliary granule from the memory
            % horizon. Recomputing its sample mean and covariance is the
            % direct windowed implementation and avoids a second, unrelated
            % exponential tracker recursion.

            if isempty(obj.zetaWindow)
                return;
            end

            obj.tracker_nu = mean(obj.zetaWindow, 2);
            if size(obj.zetaWindow, 2) >= 2
                Sigma_aux = cov(obj.zetaWindow');
            else
                Sigma_aux = eye(obj.nzeta);
            end
            Sigma_aux = obj.makePositiveDefinite(Sigma_aux);
            obj.tracker_C = obj.makePositiveDefinite(pinv(Sigma_aux));
            obj.tracker_initialized = true;
        end

        function create = shouldCreateNewGranule(obj)
            %SHOULDCREATENEWGRANULE Decide whether a new granule must be created.
            %
            % Conditions:
            %   1) anomaly_counter > n_anomaly_max
            %   2) if enabled, auxiliary tracker is c-separated from all granules
            %
            % This implements the two creation conditions in Section 2.2.

            create = false;

            if obj.anomaly_counter <= obj.n_anomaly_max
                return;
            end

            if obj.use_c_separation && ~obj.isCSeparated()
                return;
            end

            create = true;
        end

        function separated = isCSeparated(obj)
            %ISCSEPARATED Check c-separation between G_aux and all granules.
            %
            % Paper condition:
            %   ||nu_aux - nu_i|| >= c * sqrt(nzeta * max(lambda_max(Sigma_aux), lambda_max(Sigma_i)))
            %
            % tracker_C is Sigma_aux^{-1}; therefore lambda_max(Sigma_aux)
            % is computed from the inverse covariance.

            if obj.NG == 0 || ~obj.tracker_initialized
                separated = true;
                return;
            end

            separated = true;
            sigma_aux_max = obj.largestVarianceFromInvCov(obj.tracker_C);

            for i = 1:obj.NG
                gran = obj.granules{i};
                distance = norm(obj.tracker_nu - gran.nu);
                sigma_i_max = obj.largestVarianceFromCov(gran.Sigma);

                threshold = obj.c_separation * sqrt(obj.nzeta * max(sigma_aux_max, sigma_i_max));

                if distance < threshold
                    separated = false;
                    return;
                end
            end
        end

        function createFirstGranuleFromWindow(obj)
            %CREATEFIRSTGRANULEFROMWINDOW Create the initial granule from buffered data.

            zeta0 = obj.zetaWindow(:, end);
            xnext0 = obj.xNextWindow(:, end);

            gran = TSGranule(obj.nx, obj.nu_in, zeta0, xnext0, obj.granule_params);
            obj.initializeGranuleAntecedentFromWindow(gran, obj.zetaWindow);
            gran.initConsequentWLS(obj.zetaWindow, obj.xNextWindow);

            obj.NG = 1;
            obj.granules{1} = gran;
            obj.membershipMass = size(obj.zetaWindow, 2);
            obj.performanceIndex = 0;
        end

        function createGranuleFromAnomaliesOrWindow(obj)
            %CREATEGRANULEFROMANOMALIESORWINDOW Create a new granule.
            %
            % Both antecedent and consequent are initialized from the same
            % last-phi-sample memory used to construct G_aux in the paper.

            Zinit = obj.zetaWindow;

            zeta0 = Zinit(:, end);
            xnext0 = obj.xNextWindow(:, end);

            gran = TSGranule(obj.nx, obj.nu_in, zeta0, xnext0, obj.granule_params);
            obj.initializeGranuleAntecedentFromWindow(gran, Zinit);
            gran.initConsequentWLS(obj.zetaWindow, obj.xNextWindow);

            obj.NG = obj.NG + 1;
            obj.granules{obj.NG} = gran;
            obj.membershipMass(obj.NG, 1) = size(Zinit, 2);
            obj.performanceIndex(obj.NG, 1) = 0;
        end

        function initializeGranuleAntecedentFromWindow(obj, gran, Zeta)
            %INITIALIZEGRANULEANTECEDENTFROMWINDOW Initialize nu, Sigma, S/T and bounds.
            %
            % This replaces old gran_init(p, buffer) with explicit variables:
            %   nu        <- mean of Zeta
            %   Sigma     <- covariance of Zeta
            %   S, T      <- conditional dispersion matrices from (16)-(17)
            %   zeta_min  <- lower bound
            %   zeta_max  <- upper bound

            if isempty(Zeta)
                return;
            end

            N = size(Zeta, 2);
            gran.nu = mean(Zeta, 2);

            if N >= 2
                Sigma = cov(Zeta');
            else
                Sigma = eye(obj.nzeta);
            end
            Sigma = 0.5 * (Sigma + Sigma') + obj.regularization * eye(obj.nzeta);

            gran.Sigma = Sigma;
            gran.SigmaInv = pinv(Sigma);
            gran.n_samples = N;

            S = zeros(obj.nzeta, obj.nzeta);
            T = zeros(obj.nzeta, obj.nzeta);
            s_count = zeros(obj.nzeta, 1);
            t_count = zeros(obj.nzeta, 1);

            for j = 1:obj.nzeta
                below = Zeta(j, :) < gran.nu(j);
                above = ~below;
                s_count(j) = sum(below);
                t_count(j) = sum(above);

                if s_count(j) > 0
                    S(j, :) = mean(gran.nu - Zeta(:, below), 2)';
                end
                if t_count(j) > 0
                    T(j, :) = mean(Zeta(:, above) - gran.nu, 2)';
                end
            end

            gran.S = S;
            gran.T = T;
            gran.s_count = s_count;
            gran.t_count = t_count;
            gran.membership_sum = max(N, 1);
            gran.membership_square_sum = max(N, 1);
            gran.refreshBounds();
        end

        function snapshot = captureAntecedent(~, gran)
            %CAPTUREANTECEDENT Save antecedent fields before a tentative update.

            snapshot.nu = gran.nu;
            snapshot.Sigma = gran.Sigma;
            snapshot.SigmaInv = gran.SigmaInv;
            snapshot.zeta_min = gran.zeta_min;
            snapshot.zeta_max = gran.zeta_max;
            snapshot.S = gran.S;
            snapshot.T = gran.T;
            snapshot.s_count = gran.s_count;
            snapshot.t_count = gran.t_count;
            snapshot.membership_sum = gran.membership_sum;
            snapshot.membership_square_sum = gran.membership_square_sum;
            snapshot.n_samples = gran.n_samples;
        end

        function restoreAntecedent(~, gran, snapshot)
            %RESTOREANTECEDENT Restore antecedent fields after a rejected update.

            gran.nu = snapshot.nu;
            gran.Sigma = snapshot.Sigma;
            gran.SigmaInv = snapshot.SigmaInv;
            gran.zeta_min = snapshot.zeta_min;
            gran.zeta_max = snapshot.zeta_max;
            gran.S = snapshot.S;
            gran.T = snapshot.T;
            gran.s_count = snapshot.s_count;
            gran.t_count = snapshot.t_count;
            gran.membership_sum = snapshot.membership_sum;
            gran.membership_square_sum = snapshot.membership_square_sum;
            gran.n_samples = snapshot.n_samples;
        end

        function M = makePositiveDefinite(obj, M)
            %MAKEPOSITIVEDEFINITE Symmetrize and floor eigenvalues.

            M = 0.5 * (M + M');
            [V, D] = eig(M);
            d = real(diag(D));
            d(~isfinite(d)) = obj.regularization;
            d = max(d, obj.regularization);
            M = real(V * diag(d) * V');
            M = 0.5 * (M + M');
        end

        function v = largestVarianceFromInvCov(obj, C)
            %LARGESTVARIANCEFROMINVCOV Largest eigenvalue of Sigma from C=Sigma^{-1}.

            C = 0.5 * (C + C') + obj.regularization * eye(size(C));
            eigC = eig(C);
            eigC = real(eigC(isfinite(eigC)));
            eigC = max(eigC, obj.regularization);
            v = 1 / min(eigC);
        end

        function v = largestVarianceFromCov(obj, Sigma)
            %LARGESTVARIANCEFROMCOV Largest eigenvalue of covariance Sigma.

            Sigma = 0.5 * (Sigma + Sigma') + obj.regularization * eye(size(Sigma));
            eigS = eig(Sigma);
            eigS = real(eigS(isfinite(eigS)));
            if isempty(eigS)
                v = 1;
            else
                v = max(eigS);
            end
        end

        function status = emptyStatus(~)
            %EMPTYSTATUS Create a standard status struct for debugging/logging.

            status = struct();
            status.mode = '';
            status.buffering = false;
            status.created_first_granule = false;
            status.created_new_granule = false;
            status.is_anomaly = false;
            status.anomaly_counter = 0;
            status.active_idx = NaN;
            status.accepted_idx = [];
            status.reverted_idx = [];
            status.g_pre = [];
            status.xi_pre = [];
            status.g_post = [];
            status.xi_post = [];
        end
    end
end

function value = getFieldOrDefault(s, field, defaultValue)
    %GETFIELDORDEFAULT Return s.field if it exists, otherwise defaultValue.

    if isstruct(s) && isfield(s, field)
        value = s.(field);
    else
        value = defaultValue;
    end
end
