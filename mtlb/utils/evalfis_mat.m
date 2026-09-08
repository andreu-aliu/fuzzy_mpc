% Returns the row vectors of the discrete model Δx = A x + b evaluated at input
% And output at the input
% Input ordering is defined by the extracted FIS model.
function [A, b, y] = evalfis_mat(fis, input)
    w = zeros(fis.n_r,1); f = zeros(fis.n_r,1);

    for i = 1:fis.n_r
        % Firing strenght of rule i
        mu = zeros(fis.n_in, 1);
        for j = 1:fis.n_in
            sigma = fis.mf{j,i}(1);
            c = fis.mf{j,i}(2);
            mu(j) = exp(-((input(j) - c).^2) ./ (2*sigma^2));
        end
        w(i) = prod(mu);
    
        % Rule output
        f(i) = fis.A{i} * input' + fis.b{i};
    end
    
    % Normalized weights (robust)
    w_sum = sum(w);
    if w_sum <= 1e-12 || ~isfinite(w_sum)
        [w_max, i_max] = max(w);
        if isfinite(w_max) && w_max > 0
            w_n = zeros(fis.n_r,1);
            w_n(i_max) = 1;
        else
            w_n = ones(fis.n_r,1) / fis.n_r;
        end
    else
        w_n = w / w_sum;
    end

    % Total sum
    y = w_n' * f;
    if(isnan(y))
        fprintf("ANFIS input is outside all membership functions");
    end
    
    % Calculate row matrixes (A, b)
    A = zeros(1, fis.n_in);
    b = 0;
    for i = 1:fis.n_r
        A = A + w_n(i) * fis.A{i};
        b = b + w_n(i) * fis.b{i};
    end
end
