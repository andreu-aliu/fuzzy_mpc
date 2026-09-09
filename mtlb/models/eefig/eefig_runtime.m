function output = eefig_runtime(action, varargin)
%EEFIG_RUNTIME Own the persistent, optionally adaptive EEFig model instance.

persistent cached_model;
action = validatestring(action, {'get','update','reset','model_file'});
model_file = fullfile(fileparts(mfilename('fullpath')), 'eefig.mat');

switch action
    case 'reset'
        cached_model = [];
        output = [];
        return;
    case 'model_file'
        output = model_file;
        return;
end

if isempty(cached_model)
    if ~isfile(model_file)
        error(['EEFIG model not found: %s\nRun ' ...
            'models/eefig/eefig_training.m first.'], model_file);
    end
    loaded = load(model_file, 'eefig_model');
    if ~isfield(loaded, 'eefig_model') || ...
            ~isfield(loaded.eefig_model, 'learner')
        error('Invalid EEFIG model file: %s', model_file);
    end
    cached_model = loaded.eefig_model;
end

switch action
    case 'get'
        output = cached_model;

    case 'update'
        x = varargin{1}(:);
        u = varargin{2}(:);
        x_next = varargin{3}(:);
        validateattributes(x, {'numeric'}, {'numel',2,'finite'});
        validateattributes(u, {'numeric'}, {'numel',2,'finite'});
        validateattributes(x_next, {'numeric'}, {'numel',2,'finite'});

        x_n = x ./ cached_model.norm.scale_x;
        u_n = u ./ cached_model.norm.scale_u;
        x_next_n = x_next ./ cached_model.norm.scale_x;
        output = cached_model.learner.updateOnline(x_n, u_n, x_next_n);
end
end
