classdef TSGranule < handle
    % TSGRANULE One Takagi-Sugeno EEFIG granule G_k^i.
    %
    % This class represents one fuzzy information granule G_k^i.
    % It contains:
    %   - Antecedent part: nu, Sigma, SigmaInv, zeta_min, zeta_max
    %   - Consequent part: A, B, Theta, P_rls
    %
    % Local model:
    %   x_{k+1}^i = A_k^i x_k + B_k^i u_k
    %
    % Premise vector:
    %   zeta_k = [x_k; u_k]

    properties
        % Dimensions
        nx              % Number of states
        nu_in           % Number of control inputs
        nzeta           % Dimension of premise vector zeta = [x; u]

        % Antecedent parameters of G_k^i
        nu              % Granule center nu_k^i
        Sigma           % Granule covariance matrix Sigma_k^i
        SigmaInv        % Inverse covariance matrix Sigma_k^i^{-1}

        zeta_min        % Lower bound underline{zeta}_k^i
        zeta_max        % Upper bound overline{zeta}_k^i

        S               % Lower-side conditional dispersions S_k,j,l^i
        T               % Upper-side conditional dispersions T_k,j,l^i
        s_count         % Number of samples in each lower partition Z^-_k,j
        t_count         % Number of samples in each upper partition Z^+_k,j

        membership_sum          % First accumulated membership moment
        membership_square_sum   % Second accumulated membership moment

        n_samples       % Number of samples admitted into this granule

        % Consequent parameters of G_k^i
        A               % Local state matrix A_k^i
        B               % Local input matrix B_k^i
        Theta           % Regression matrix Theta = [A B], size nx x nzeta

        P_rls           % RLS covariance matrix, size nzeta x nzeta

        % Hyperparameters
        lambda          % Granule bound expansion factor, usually in [2, 4]
        iota            % Specificity contraction factor, usually in [0, 1]
        betaS           % Lower admissible bound
        betaT           % Upper admissible bound
        eta             % RLS forgetting factor
        reg             % Small regularization value
    end

    methods
        function obj = TSGranule(nx, nu_in, zeta0, x_next0, params)
            % TSGRANULE Construct one granule initialized from one sample.
            %
            % Inputs:
            %   nx       - number of states
            %   nu_in    - number of control inputs
            %   zeta0    - initial premise vector [x_k; u_k]
            %   x_next0  - next state x_{k+1}, used for initial consequent
            %   params   - struct with lambda, iota, betaS, betaT, eta, reg

            obj.nx = nx;
            obj.nu_in = nu_in;
            obj.nzeta = nx + nu_in;

            zeta0 = zeta0(:);
            x_next0 = x_next0(:);

            obj.lambda = getFieldOrDefault(params, 'lambda', 3.0);
            obj.iota   = getFieldOrDefault(params, 'iota',   0.3);
            % betaS=0 in the paper because its premise variables are mapped
            % to a non-negative domain. Vehicle states contain signed values,
            % so -Inf is the safe default unless the caller scales them.
            obj.betaS  = getFieldOrDefault(params, 'betaS', -inf);
            obj.betaT  = getFieldOrDefault(params, 'betaT',  inf);
            obj.eta    = getFieldOrDefault(params, 'eta',    0.99);
            obj.reg    = getFieldOrDefault(params, 'reg',    1e-8);

            obj.nu = zeta0;
            obj.Sigma = eye(obj.nzeta) * obj.reg;
            obj.SigmaInv = inv(obj.Sigma);

            obj.S = zeros(obj.nzeta, obj.nzeta);
            obj.T = zeros(obj.nzeta, obj.nzeta);
            obj.s_count = zeros(obj.nzeta, 1);
            obj.t_count = zeros(obj.nzeta, 1);
            obj.membership_sum = 1;
            obj.membership_square_sum = 1;

            obj.zeta_min = obj.nu - sqrt(obj.reg);
            obj.zeta_max = obj.nu + sqrt(obj.reg);

            obj.n_samples = 1;

            obj.Theta = zeros(obj.nx, obj.nzeta);

            % Simple initial fit:
            % For each state l, solve x_next_l ≈ theta_l^T zeta0.
            denom = zeta0' * zeta0 + obj.reg;
            obj.Theta = x_next0 * zeta0' / denom;

            obj.A = obj.Theta(:, 1:obj.nx);
            obj.B = obj.Theta(:, obj.nx+1:end);

            obj.P_rls = eye(obj.nzeta) * 1e5;
        end

        function M = mahalanobis(obj, zeta)
            % MAHALANOBIS Compute M_k^i for the ellipsoid E_k^i.
            %
            % M_k^i = (zeta_k - nu_k^i)' * Sigma_k^i^{-1} * (zeta_k - nu_k^i)

            zeta = zeta(:);
            dz = zeta - obj.nu;
            M = dz' * obj.SigmaInv * dz;
        end

        function inside = contains(obj, zeta, epsilon)
            % CONTAINS Check whether zeta_k belongs to E_k^i.
            %
            % E_k^i = { zeta_k in Z_k | M_k^i <= epsilon }

            M = obj.mahalanobis(zeta);
            inside = (M <= epsilon);
        end

        function xi = membership(obj, zeta)
            % MEMBERSHIP Compute non-normalized membership xi_k^i(zeta_k).
            %
            % This follows the idea of equation (10):
            % xi_k^i(zeta_k) depends on the distance from zeta_k to nu_k^i,
            % normalized by the granule bounds.

            zeta = zeta(:);

            width = obj.zeta_max - obj.zeta_min;
            width = max(width, obj.reg);

            normalized_distance = sqrt(sum(((zeta - obj.nu) ./ width).^2));

            xi = exp(-2.0 * normalized_distance);
        end

        function updateAntecedent(obj, zeta, g_i)
            % UPDATEANTECEDENT Update nu, Sigma, S, T, zeta_min and zeta_max.
            %
            % This updates the antecedent part of G_k^i after zeta_k has
            % been admitted into the granule.
            %
            % g_i is the normalized membership degree g_k^i(zeta_k).

            zeta = zeta(:);

            nu_old = obj.nu;
            C_old = obj.SigmaInv;

            weight = min(max(double(g_i), obj.reg), 1);
            alpha_old = max(obj.membership_sum, 1);
            beta_old = max(obj.membership_square_sum, obj.reg);
            alpha_new = alpha_old + weight;
            beta_new = beta_old + weight^2;

            % Update the centre with the normalized accumulated evidence.
            % This is the recursive-mean realization of (15) used by the
            % underlying EEFIG method; without the accumulated denominator,
            % a single-granule model would jump completely to every sample.
            center_gain = weight / alpha_old;
            obj.nu = nu_old + center_gain * (zeta - nu_old);

            % Equation (18): recursive inverse-covariance update. The first
            % and second accumulated membership moments correspond to rho
            % and tau in the paper's Gamma/Lambda expressions. The algebraic
            % form below follows the reference EEFIG recursion.
            gamma_num = alpha_old * (alpha_new^2 - beta_new);
            gamma_den = alpha_new * (alpha_old^2 - beta_new);
            lambda_num = alpha_new * (alpha_old^2 - beta_old);
            lambda_den = alpha_old * weight * (weight + alpha_new - 2);

            Gamma = gamma_num / gamma_den;
            Lambda = lambda_num / lambda_den;
            dz = zeta - obj.nu;
            sm_denom = Lambda + dz' * C_old * dz;

            if isfinite(Gamma) && Gamma > 0 && isfinite(sm_denom) && sm_denom > obj.reg
                C_new = Gamma * (C_old - (C_old * (dz * dz') * C_old) / sm_denom);
            else
                % Degenerate first-moment configurations can make Eq. (18)
                % undefined. Retain the previous ellipsoid instead of
                % injecting NaN/Inf into every subsequent membership.
                C_new = C_old;
            end

            C_new = obj.makePositiveDefinite(C_new);
            obj.SigmaInv = C_new;
            obj.Sigma = obj.makePositiveDefinite(pinv(C_new));

            % Equations (16)-(17). Each row j stores a complete dispersion
            % vector conditioned on whether premise component j lies below
            % or above the updated centre; cross-feature information is kept.
            for j = 1:obj.nzeta
                if zeta(j) < obj.nu(j)
                    obj.s_count(j) = obj.s_count(j) + 1;
                    n = obj.s_count(j);
                    obj.S(j, :) = ((n - 1) * obj.S(j, :) + (obj.nu - zeta)') / n;
                else
                    obj.t_count(j) = obj.t_count(j) + 1;
                    n = obj.t_count(j);
                    obj.T(j, :) = ((n - 1) * obj.T(j, :) + (zeta - obj.nu)') / n;
                end
            end

            obj.membership_sum = alpha_new;
            obj.membership_square_sum = beta_new;
            obj.n_samples = obj.n_samples + 1;
            obj.refreshBounds();
        end

        function refreshBounds(obj)
            % REFRESHBOUNDS Compute (6)-(7) from conditional dispersions.
            %
            % The original PJG optimizer selects bounds from the conditional
            % candidates indexed by j. Taking the largest positive candidate
            % independently in each dimension preserves their union and keeps
            % a single lower/upper vector required by equation (10).

            lower_spread = max(max(obj.S, [], 1)', 0);
            upper_spread = max(max(obj.T, [], 1)', 0);

            lower_raw = max(obj.nu - obj.lambda * lower_spread, obj.betaS);
            upper_raw = min(obj.nu + obj.lambda * upper_spread, obj.betaT);

            obj.zeta_min = lower_raw + (obj.nu - lower_raw) * obj.iota;
            obj.zeta_max = upper_raw - (upper_raw - obj.nu) * obj.iota;

            min_half_width = sqrt(obj.reg);
            obj.zeta_min = min(obj.zeta_min, obj.nu - min_half_width);
            obj.zeta_max = max(obj.zeta_max, obj.nu + min_half_width);
        end

        function updateConsequentRLS(obj, zeta_prev, x_next)
            % UPDATECONSEQUENTRLS Update A_k^i and B_k^i using RLS.
            %
            % Regression:
            %   x_{k+1} = Theta_k * zeta_k
            %
            % where:
            %   Theta_k = [A_k^i  B_k^i]
            %
            % This is the consequent update associated with equations
            % (21)-(23) in the paper.

            phi = zeta_prev(:);
            y = x_next(:);

            % RLS gain.
            K = (obj.P_rls * phi) / (obj.eta + phi' * obj.P_rls * phi);

            % Prediction error.
            y_hat = obj.Theta * phi;
            e = y - y_hat;

            % Parameter update.
            obj.Theta = obj.Theta + e * K';

            % RLS covariance update.
            obj.P_rls = (1 / obj.eta) * (eye(obj.nzeta) - K * phi') * obj.P_rls;

            % Keep numerical symmetry.
            obj.P_rls = 0.5 * (obj.P_rls + obj.P_rls');

            % Extract A and B.
            obj.A = obj.Theta(:, 1:obj.nx);
            obj.B = obj.Theta(:, obj.nx+1:end);
        end

        function initConsequentWLS(obj, Zeta_prev_window, X_next_window)
            % INITCONSEQUENTWLS Initialize A_k^i and B_k^i using WLS/LS.
            %
            % Used when a new granule is created from a window of phi samples.
            %
            % Inputs:
            %   Zeta_prev_window : nzeta x N matrix
            %       columns are zeta_{k-1}, zeta_{k-2}, ...
            %
            %   X_next_window : nx x N matrix
            %       columns are x_k, x_{k-1}, ...
            %
            % Solves:
            %   X_next_window ≈ Theta * Zeta_prev_window

            Phi = Zeta_prev_window;
            Y = X_next_window;

            obj.Theta = Y * pinv(Phi);

            obj.A = obj.Theta(:, 1:obj.nx);
            obj.B = obj.Theta(:, obj.nx+1:end);
        end

        function x_next_pred = predictLocal(obj, x, u)
            % PREDICTLOCAL Predict next state using the local TS consequent.
            %
            % x_{k+1}^i = A_k^i x_k + B_k^i u_k

            x = x(:);
            u = u(:);

            x_next_pred = obj.A * x + obj.B * u;
        end

        function M = makePositiveDefinite(obj, M)
            % MAKEPOSITIVEDEFINITE Symmetrize and floor eigenvalues.

            M = 0.5 * (M + M');
            [V, D] = eig(M);
            d = real(diag(D));
            d(~isfinite(d)) = obj.reg;
            d = max(d, obj.reg);
            M = real(V * diag(d) * V');
            M = 0.5 * (M + M');
        end
    end
end

function value = getFieldOrDefault(s, field, defaultValue)
    % GETFIELDORDEFAULT Read struct field or return default value.

    if isfield(s, field)
        value = s.(field);
    else
        value = defaultValue;
    end
end
