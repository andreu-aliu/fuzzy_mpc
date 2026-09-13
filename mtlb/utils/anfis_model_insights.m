function figure_handles = anfis_model_insights( ...
        fis,Xn,trainError,valError,varargin)

p = inputParser;
p.addParameter('inputLabels', {}, @(c) iscell(c) || isstring(c));
p.addParameter('titlePrefix', "", @(s) ischar(s) || isstring(s));
p.addParameter('figBase', 0, @(x) isnumeric(x) && isscalar(x));
p.parse(varargin{:});

inputLabels = cellstr(p.Results.inputLabels);
titlePrefix = string(p.Results.titlePrefix);
figBase = p.Results.figBase;

try
    nIn = numel(fis.Inputs);
catch
    nIn = fis.NumInputs;
end

if isempty(inputLabels)
    if nIn == 4
        inputLabels = {'vy','r','vx','delta'};
    else
        inputLabels = arrayfun(@(k) sprintf('in%d', k), 1:nIn, 'UniformOutput', false);
    end
end

if ~isempty(titlePrefix)
    titlePrefix = titlePrefix + " - ";
end

fprintf("Number of rules: %d\n", numel(fis.Rules))
showrule(fis)

figure_handles = gobjects(nIn+1,1);
for j = 1:nIn
    figure_name = titlePrefix+inputLabels{j}+" membership";
    figure_handles(j) = figure(figBase+j);
    figure_handles(j).Name = char(figure_name);
    plotmf(fis,'input',j);
    title(figure_name);
    hold on;
    if ~isempty(Xn) && size(Xn,2) >= j
        histogram(Xn(:,j), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none');
    end
    hold off;
end

figure_handles(nIn+1) = figure(figBase+nIn+1);
figure_handles(nIn+1).Name = char(titlePrefix+"ANFIS Training Error");
plot(trainError); hold on;
plot(valError); hold off;
xlabel('Epoch');
ylabel('Training RMSE');
grid on;
title(titlePrefix + "ANFIS Training Error");
ylim([0, max([trainError; valError])])
legend('Training Error', 'Validation Error')

fprintf('Model trained to %f factor of RMSE\n', trainError(end)/trainError(1))
end
