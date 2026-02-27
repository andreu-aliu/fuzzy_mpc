function y = sgolayfilt_custom(x, polyOrder, frameLen)
%SGOLAYFILT_CUSTOM Savitzky-Golay smoothing filter (no toolboxes)
%
%   y = sgolayfilt_custom(x, polyOrder, frameLen)
%
%   x         : input signal (column or row vector)
%   polyOrder : polynomial order (e.g. 2 or 3)
%   frameLen  : odd window length (e.g. 21, 31)
%
%   This performs local least-squares polynomial fitting
%   and returns the smoothed signal (0th derivative).

    % --- checks ---
    if mod(frameLen,2) == 0
        error('frameLen must be odd');
    end
    if polyOrder >= frameLen
        error('polyOrder must be < frameLen');
    end

    x = x(:);                 % force column
    N = length(x);
    half = floor(frameLen/2);

    % --- build Vandermonde matrix ---
    t = (-half:half)';
    A = zeros(frameLen, polyOrder+1);
    for k = 0:polyOrder
        A(:,k+1) = t.^k;
    end

    % --- least-squares projection matrix ---
    % This gives convolution coefficients for smoothing
    ATA_inv = inv(A' * A);
    B = ATA_inv * A';
    h = B(1,:);               % 0th derivative coefficients

    % --- apply convolution ---
    y = zeros(N,1);
    for i = 1:N
        i1 = max(1, i-half);
        i2 = min(N, i+half);

        % adjust kernel near edges
        k1 = half+1 - (i - i1);
        k2 = half+1 + (i2 - i);

        y(i) = h(k1:k2) * x(i1:i2);
    end
end
