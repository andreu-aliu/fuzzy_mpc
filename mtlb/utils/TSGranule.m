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

        S               % Lower-side dispersion estimate S_k^i
        T               % Upper-side dispersion estimate T_k^i

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

            obj.lambda = getFieldOrDefault(params, 'lambda', 4.0);
            obj.iota   = getFieldOrDefault(params, 'iota',   0.0);
            obj.betaS  = getFieldOrDefault(params, 'betaS',  0.0);
            obj.betaT  = getFieldOrDefault(params, 'betaT',  inf);
            obj.eta    = getFieldOrDefault(params, 'eta',    0.99);
            obj.reg    = getFieldOrDefault(params, 'reg',    1e-6);

            obj.nu = zeta0;
            obj.Sigma = eye(obj.nzeta) * obj.reg;
            obj.SigmaInv = inv(obj.Sigma);

            obj.S = zeros(obj.nzeta, 1);
            obj.T = zeros(obj.nzeta, 1);

            obj.zeta_min = max(obj.nu - obj.lambda * obj.S, obj.betaS);
            obj.zeta_max = min(obj.nu + obj.lambda * obj.T, obj.betaT);

            obj.n_samples = 1;

            obj.Theta = zeros(obj.nx, obj.nzeta);

            % Simple initial fit:
            % For each state l, solve x_next_l ≈ theta_l^T zeta0.
            denom = zeta0' * zeta0 + obj.reg;
            obj.Theta = x_next0 * zeta0' / denom;

            obj.A = obj.Theta(:, 1:obj.nx);
            obj.B = obj.Theta(:, obj.nx+1:end);

            obj.P_rls = eye(obj.nzeta) * 1e3;
        end

        function M = mahalanobis(obj, zeta)
            %MAHALANOBIS Compute M_k^i for the ellipsoid E_k^i.
            %
            % M_k^i = (zeta_k - nu_k^i)' * Sigma_k^i^{-1} * (zeta_k - nu_k^i)

            zeta = zeta(:);
            dz = zeta - obj.nu;
            M = dz' * obj.SigmaInv * dz;
        end

        function inside = contains(obj, zeta, epsilon)
            %CONTAINS Check whether zeta_k belongs to E_k^i.
            %
            % E_k^i = { zeta_k in Z_k | M_k^i <= epsilon }

            M = obj.mahalanobis(zeta);
            inside = (M <= epsilon);
        end

        function xi = membership(obj, zeta)
            %MEMBERSHIP Compute non-normalized membership xi_k^i(zeta_k).
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

        function updateAntecedent(obj, zeta, g_i) % TODO: Not the same as paper
            %UPDATEANTECEDENT Update nu, Sigma, S, T, zeta_min and zeta_max.
            %
            % This updates the antecedent part of G_k^i after zeta_k has
            % been admitted into the granule.
            %
            % g_i is the normalized membership degree g_k^i(zeta_k).

            zeta = zeta(:);

            obj.n_samples = obj.n_samples + 1;

            nu_old = obj.nu;
            Sigma_old = obj.Sigma;

            % Update center nu_k^i.
            % Paper-inspired incremental update:
            % nu_k^i = nu_{k-1}^i + g_i * (zeta_k - nu_{k-1}^i)
            obj.nu = nu_old + g_i * (zeta - nu_old);

            % Update covariance with a stable weighted incremental formula.
            % This is not a literal copy of eq. (18), but it serves the same
            % purpose: updating Sigma_k^i online from admitted samples.
            dz_old = zeta - nu_old;
            dz_new = zeta - obj.nu;

            alpha = 1.0 / obj.n_samples;
            obj.Sigma = (1 - alpha) * Sigma_old + alpha * (dz_old * dz_new');

            % Regularize to avoid singular covariance.
            obj.Sigma = 0.5 * (obj.Sigma + obj.Sigma') + obj.reg * eye(obj.nzeta);
            obj.SigmaInv = inv(obj.Sigma);

            % Update lower/upper side dispersion estimates.
            for j = 1:obj.nzeta
                if zeta(j) < obj.nu(j)
                    obj.S(j) = obj.S(j) + alpha * ((obj.nu(j) - zeta(j)) - obj.S(j));
                else
                    obj.T(j) = obj.T(j) + alpha * ((zeta(j) - obj.nu(j)) - obj.T(j));
                end
            end

            % Update bounds underline{zeta}_k^i and overline{zeta}_k^i.
            lower_raw = max(obj.nu - obj.lambda * obj.S, obj.betaS);
            upper_raw = min(obj.nu + obj.lambda * obj.T, obj.betaT);

            obj.zeta_min = lower_raw + (obj.nu - lower_raw) * obj.iota;
            obj.zeta_max = upper_raw - (upper_raw - obj.nu) * obj.iota;
        end

        function updateConsequentRLS(obj, zeta_prev, x_next)
            %UPDATECONSEQUENTRLS Update A_k^i and B_k^i using RLS.
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
            %INITCONSEQUENTWLS Initialize A_k^i and B_k^i using WLS/LS.
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
            %PREDICTLOCAL Predict next state using the local TS consequent.
            %
            % x_{k+1}^i = A_k^i x_k + B_k^i u_k

            x = x(:);
            u = u(:);

            x_next_pred = obj.A * x + obj.B * u;
        end
    end
end

function value = getFieldOrDefault(s, field, defaultValue)
    %GETFIELDORDEFAULT Read struct field or return default value.

    if isfield(s, field)
        value = s.(field);
    else
        value = defaultValue;
    end
end