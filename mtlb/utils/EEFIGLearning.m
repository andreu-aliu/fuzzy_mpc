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
        membershipMass          % Running sum of normalized memberships per granule, used by the PJG-like quality test

        % Ellipsoidal admission and granule update parameters
        epsilon                 % Mahalanobis threshold defining E_k^i = {zeta | M_k^i <= epsilon}
        confidence              % Optional chi-square confidence used to compute epsilon
        membership_update_threshold % Old-code style threshold; granules with g_i below this are not tested for update
        use_pjg_quality_check   % If true, revert antecedent updates that reduce the granule quality index

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
        tracker_forgetting      % Exponential forgetting factor used by the old tracker update
        tracker_effective_N     % Effective sample count used in the old inverse-covariance tracker update
        c_separation            % c coefficient in the c-separation condition
        use_c_separation        % If true, require c-separation before creating a new granule

        % Consequent learning parameters
        rls_forgetting          % RLS forgetting factor eta/ff
        rls_mode                % 'per_granule' follows the paper; 'global' reproduces the old shared P behaviour
        global_P                % Global RLS covariance used only when rls_mode = 'global'
        global_K                % Last global RLS gain used only when rls_mode = 'global'

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

            obj.confidence = getFieldOrDefault(params, 'confidence', 0.95);
            obj.epsilon = getFieldOrDefault(params, 'epsilon', chi2inv(obj.confidence, obj.nzeta));
            obj.membership_update_threshold = getFieldOrDefault(params, 'membership_update_threshold', 1e-6);
            obj.use_pjg_quality_check = getFieldOrDefault(params, 'use_pjg_quality_check', false);

            obj.phi = getFieldOrDefault(params, 'phi', 30);
            obj.n_anomaly_max = getFieldOrDefault(params, 'n_anomaly_max', obj.nzeta);
            obj.anomaly_counter = 0;
            obj.anomalyZetaBuffer = zeros(obj.nzeta, 0);
            obj.anomalyXNextBuffer = zeros(obj.nx, 0);
            obj.clear_anomaly_buffer_on_normal = getFieldOrDefault(params, 'clear_anomaly_buffer_on_normal', false);

            obj.tracker_nu = zeros(obj.nzeta, 1);
            obj.tracker_C = eye(obj.nzeta);
            obj.tracker_initialized = false;
            obj.tracker_forgetting = getFieldOrDefault(params, 'tracker_forgetting', 0.99);
            obj.tracker_effective_N = getFieldOrDefault(params, 'tracker_effective_N', 200);
            obj.c_separation = getFieldOrDefault(params, 'c_separation', 2.0);
            obj.use_c_separation = getFieldOrDefault(params, 'use_c_separation', true);

            obj.rls_forgetting = getFieldOrDefault(params, 'rls_forgetting', 0.975);
            obj.rls_mode = getFieldOrDefault(params, 'rls_mode', 'per_granule');
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
            %   - In 'offline' mode, consequents are preferably fitted later
            %     with fitAllConsequentsWLS(); no RLS is applied.

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

            if obj.shouldCreateNewGranule()
                obj.createGranuleFromAnomaliesOrWindow();
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

            % Consequent update.
            if strcmpi(mode, 'online')
                obj.updateConsequentOnline(status.active_idx, zeta, x_next);
            elseif strcmpi(mode, 'offline')
                % For offline learning, the paper uses WLS. We postpone the
                % final WLS to trainOfflinePairs()/fitAllConsequentsWLS(),
                % but still initialize empty consequents from the current window.
                obj.ensureConsequentExists(status.active_idx);
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
            % This function updates the granule structure sample by sample,
            % then fits every consequent A_i,B_i with WLS using all assigned
            % offline samples.

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

            obj.fitAllConsequentsWLS(Zeta, Xnext);
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

            obj.fitAllConsequentsWLS(Zeta, Xnext);
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

        function [is_anomaly, accepted_idx, reverted_idx] = evaluateAndUpdateAntecedents(obj, zeta, g_pre)
            %EVALUATEANDUPDATEANTECEDENTS Update all admissible granule antecedents.
            %
            % This is the structured replacement of data_evaluation():
            %   1) loop over all granules,
            %   2) skip granules with negligible normalized membership,
            %   3) check ellipsoidal admission zeta in E_k^i,
            %   4) update nu, Sigma, zeta_min, zeta_max,
            %   5) optionally revert if the PJG-like quality index worsens,
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
            % The sample is accepted only if zeta belongs to the ellipsoid
            % E_k^i, i.e. M_k^i <= epsilon. If use_pjg_quality_check is true,
            % the update is kept only when the quality index does not decrease.

            accepted = false;
            reverted = false;
            gran = obj.granules{i};

            if ~gran.contains(zeta, obj.epsilon)
                return;
            end

            accepted = true;
            snapshot = obj.captureAntecedent(gran);
            oldQuality = obj.granuleQuality(i, gran, zeta, g_i);

            gran.updateAntecedent(zeta, g_i);

            newQuality = obj.granuleQuality(i, gran, zeta, g_i);

            if obj.use_pjg_quality_check && newQuality < oldQuality
                obj.restoreAntecedent(gran, snapshot);
                reverted = true;
            else
                obj.membershipMass(i) = obj.membershipMass(i) + g_i;
            end
        end

        function Q = granuleQuality(obj, i, gran, zeta, g_i)
            %GRANULEQUALITY PJG-inspired quality index for an update.
            %
            % The paper defines an index involving the Mahalanobis distance
            % and accumulated membership evidence. The old code stores this
            % as Q and gsum. Here we keep the same idea at the manager level:
            %   Q_i(zeta) = M_i(zeta) * accumulated_membership_i.
            %
            % A larger value means that the granule covers more evidence for
            % the admitted sample. This is used only if use_pjg_quality_check=true.

            M = gran.mahalanobis(zeta);
            Q = M * max(obj.membershipMass(i) + g_i, obj.regularization);
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
            %UPDATECONSEQUENTRLSGLOBALP Old-code style RLS with one shared P.
            %
            % The old learn_EEFIG code used a single covariance matrix P and
            % gain K for all granules. This method preserves that behaviour.

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
            %FITALLCONSEQUENTSWLS Offline WLS fit for every granule.
            %
            % Each sample is assigned to the granule with maximum current
            % membership. Then each local model is fitted using the samples
            % assigned to that granule. If a granule receives too few samples,
            % the full dataset is used as a fallback to keep A/B defined.

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

        function updateAuxTracker(obj, zeta)
            %UPDATEAUXTRACKER Update auxiliary tracker from the newest sample.
            %
            % This follows the spirit of the old trackerm/trackerC update.
            % tracker_C stores an inverse covariance, not Sigma directly.

            zeta = zeta(:);

            if ~obj.tracker_initialized
                obj.tracker_nu = zeta;
                obj.tracker_C = eye(obj.nzeta);
                obj.tracker_initialized = true;
                return;
            end

            prev_nu = obj.tracker_nu;
            prev_C = obj.tracker_C;
            lambda = obj.tracker_forgetting;
            effectiveN = obj.tracker_effective_N;

            dz = zeta - prev_nu;
            denom = dz' * prev_C * dz + (effectiveN - 1) / lambda;
            multiplier = effectiveN / ((effectiveN - 1) * lambda);

            C_new = prev_C - (prev_C * dz * dz' * prev_C) / max(denom, obj.regularization);
            C_new = multiplier * C_new;
            C_new = 0.5 * (C_new + C_new') + obj.regularization * eye(obj.nzeta);

            if any(isnan(C_new), 'all') || any(isinf(C_new), 'all')
                C_new = eye(obj.nzeta);
            end

            obj.tracker_C = C_new;
            obj.tracker_nu = lambda * prev_nu + (1 - lambda) * zeta;
        end

        function create = shouldCreateNewGranule(obj)
            %SHOULDCREATENEWGRANULE Decide whether a new granule must be created.
            %
            % Conditions:
            %   1) anomaly_counter > n_anomaly_max
            %   2) if enabled, auxiliary tracker is c-separated from all granules
            %
            % This combines the paper text with the explicit old-code
            % c-separation check.

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
        end

        function createGranuleFromAnomaliesOrWindow(obj)
            %CREATEGRANULEFROMANOMALIESORWINDOW Create a new granule.
            %
            % Antecedent initialization:
            %   - preferably from the latest consecutive anomalous samples,
            %   - fallback to the global phi-window.
            %
            % Consequent initialization:
            %   - WLS using the current phi-window, as in the paper/old code.

            if size(obj.anomalyZetaBuffer, 2) >= max(2, obj.anomaly_counter)
                nUse = min(obj.anomaly_counter, size(obj.anomalyZetaBuffer, 2));
                Zinit = obj.anomalyZetaBuffer(:, end-nUse+1:end);
            else
                Zinit = obj.zetaWindow;
            end

            zeta0 = Zinit(:, end);
            if ~isempty(obj.anomalyXNextBuffer)
                xnext0 = obj.anomalyXNextBuffer(:, end);
            else
                xnext0 = obj.xNextWindow(:, end);
            end

            gran = TSGranule(obj.nx, obj.nu_in, zeta0, xnext0, obj.granule_params);
            obj.initializeGranuleAntecedentFromWindow(gran, Zinit);
            gran.initConsequentWLS(obj.zetaWindow, obj.xNextWindow);

            obj.NG = obj.NG + 1;
            obj.granules{obj.NG} = gran;
            obj.membershipMass(obj.NG, 1) = size(Zinit, 2);
        end

        function initializeGranuleAntecedentFromWindow(obj, gran, Zeta)
            %INITIALIZEGRANULEANTECEDENTFROMWINDOW Initialize nu, Sigma, S/T and bounds.
            %
            % This replaces old gran_init(p, buffer) with explicit variables:
            %   nu        <- mean of Zeta
            %   Sigma     <- covariance of Zeta
            %   S, T      <- average lower/upper dispersion around nu
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

            S = zeros(obj.nzeta, 1);
            T = zeros(obj.nzeta, 1);

            for j = 1:obj.nzeta
                below = Zeta(j, Zeta(j, :) < gran.nu(j));
                above = Zeta(j, Zeta(j, :) >= gran.nu(j));

                if isempty(below)
                    S(j) = obj.regularization;
                else
                    S(j) = mean(gran.nu(j) - below);
                end

                if isempty(above)
                    T(j) = obj.regularization;
                else
                    T(j) = mean(above - gran.nu(j));
                end
            end

            gran.S = S;
            gran.T = T;

            lambda = getFieldOrDefault(obj.granule_params, 'lambda', 4.0);
            iota = getFieldOrDefault(obj.granule_params, 'iota', 0.0);
            betaS = getFieldOrDefault(obj.granule_params, 'betaS', -inf);
            betaT = getFieldOrDefault(obj.granule_params, 'betaT', inf);

            lower_raw = max(gran.nu - lambda * S, betaS);
            upper_raw = min(gran.nu + lambda * T, betaT);

            gran.zeta_min = lower_raw + (gran.nu - lower_raw) * iota;
            gran.zeta_max = upper_raw - (upper_raw - gran.nu) * iota;
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
            gran.n_samples = snapshot.n_samples;
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