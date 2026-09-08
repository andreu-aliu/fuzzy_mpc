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

    inputWasRow = isrow(x);
    x = x(:);                 % use a column internally
    N = length(x);
    half = floor(frameLen/2);

    if N < frameLen
        error('Input length (%d) must be at least frameLen (%d).', N, frameLen);
    end

    % --- build Vandermonde matrix ---
    t = (-half:half)';
    A = zeros(frameLen, polyOrder+1);
    for k = 0:polyOrder
        A(:,k+1) = t.^k;
    end

    % Interior convolution coefficients for the centered polynomial fit.
    B = pinv(A);
    h = B(1,:);

    % Apply the centered filter where the full symmetric window exists.
    y = conv(x, h, 'same');

    % At each boundary, fit a full asymmetric window and evaluate the
    % polynomial at the current sample. Truncating the centered kernel here
    % would not preserve even a constant signal.
    edgeIndices = [1:half, (N-half+1):N];
    for i = edgeIndices
        if i <= half
            sampleIndices = 1:frameLen;
        else
            sampleIndices = (N-frameLen+1):N;
        end
        tLocal = sampleIndices(:) - i;
        ALocal = zeros(frameLen, polyOrder+1);
        for k = 0:polyOrder
            ALocal(:,k+1) = tLocal.^k;
        end
        coefficients = ALocal \ x(sampleIndices);
        y(i) = coefficients(1);
    end

    if inputWasRow
        y = y.';
    end
end
