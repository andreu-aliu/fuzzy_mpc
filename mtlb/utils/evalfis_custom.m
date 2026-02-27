function y = evalfis_custom(fis, input)
    w = zeros(fis.n_r,1); f = zeros(fis.n_r,1);
    for i = 1:fis.n_r
    
        % Quant pertany l'input a la rule
        mu = zeros(fis.n_in, 1);
        for j = 1:fis.n_in
            sigma = fis.mf{j,i}(1);
            c = fis.mf{j,i}(2);
            mu(j) = exp(-((input(j) - c).^2) ./ (2*sigma^2));
        end
        w(i) = prod(mu);
    
        % Quin es l'output de la rule
        f(i) = fis.A{i} * input' + fis.b{i};
    end
    
    % Normalitza funcions d'activació
    w_n = w / sum(w);
    
    % Suma total
    y = w_n' * f;
end