% This script saves the anfis model matrices to yaml file format

model_path = '../models/anfis_direct';
model_list = {direct_anfis.r, direct_anfis.vy};
model_names = {"r", "vy"};

input_norm = direct_anfis.norm;

if ~exist(model_path, 'dir')
    mkdir(model_path);
end

%% Save normalization parameters

norm_file = fullfile(model_path, "normalization.yaml");
fid = fopen(norm_file, 'w');

fprintf(fid,"normalization:\n");

fprintf(fid,"  mu: [");
for k = 1:length(input_norm.mu)
    if k < length(input_norm.mu)
        fprintf(fid,"%f, ", input_norm.mu(k));
    else
        fprintf(fid,"%f",  input_norm.mu(k));
    end
end
fprintf(fid,"]\n");

fprintf(fid,"  sigma: [");
for k = 1:length(input_norm.sigma)
    if k < length(input_norm.sigma)
        fprintf(fid,"%f, ", input_norm.sigma(k));
    else
        fprintf(fid,"%f",  input_norm.sigma(k));
    end
end
fprintf(fid,"]\n");

fprintf(fid,"  x_min: [");
for k = 1:length(input_norm.x_min)
    if k < length(input_norm.x_min)
        fprintf(fid,"%f, ", input_norm.x_min(k));
    else
        fprintf(fid,"%f",  input_norm.x_min(k));
    end
end
fprintf(fid,"]\n");

fprintf(fid,"  x_max: [");
for k = 1:length(input_norm.x_max)
    if k < length(input_norm.x_max)
        fprintf(fid,"%f, ", input_norm.x_max(k));
    else
        fprintf(fid,"%f",  input_norm.x_max(k));
    end
end
fprintf(fid,"]\n");

fclose(fid);

%% Export each ANFIS model

for m = 1:length(model_list)

    model = model_list{m};
    fis = model.fis;

    file = fullfile(model_path, model_names{m} + ".yaml");
    fid = fopen(file,'w');

    n_inputs = numel(fis.Inputs);
    n_rules  = numel(fis.Rules);

    fprintf(fid,"n_inputs: %d\n", n_inputs);
    fprintf(fid,"n_rules: %d\n\n", n_rules);

    %% Premise parameters

    fprintf(fid,"premise:\n");

    for i = 1:n_inputs

        fprintf(fid,"  input%d:\n", i);

        mfs = fis.Inputs(i).MembershipFunctions;

        for j = 1:numel(mfs)

            params = mfs(j).Parameters;

            fprintf(fid,"    - [");

            for k = 1:length(params)
                if k < length(params)
                    fprintf(fid,"%f, ", params(k));   % comma here
                else
                    fprintf(fid,"%f", params(k));     % last element no comma
                end
            end

            fprintf(fid,"]\n");

        end
    end

    fprintf(fid,"\n");

    %% Consequent parameters

    fprintf(fid,"consequent:\n");

    for r = 1:n_rules

        params = fis.Outputs(1).MembershipFunctions(r).Parameters;

        fprintf(fid,"  - [");

        for k = 1:length(params)
            if k < length(params)
                fprintf(fid,"%f, ", params(k));
            else
                fprintf(fid,"%f", params(k));
            end
        end

        fprintf(fid,"]\n");

    end

    fclose(fid);

end

disp("ANFIS models exported successfully")