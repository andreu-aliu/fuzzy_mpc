function saved_files = save_script_figures(script_name,figure_handles)
%SAVE_SCRIPT_FIGURES Export completed figures under plots/<script_name>/.
% Existing files with the same generated name are intentionally replaced so
% the folder always contains the latest output of a script/section.

validateattributes(script_name,{'char','string'},{'scalartext'}, ...
    mfilename,'script_name');
if nargin<2
    figure_handles = findall(groot,'Type','figure');
end
figure_handles = figure_handles(isgraphics(figure_handles,'figure'));
if isempty(figure_handles)
    saved_files = strings(0,1);
    return
end

% Stable ordering makes duplicate-name suffixes reproducible.
[~,order] = sort([figure_handles.Number]);
figure_handles = figure_handles(order);

utils_dir = fileparts(mfilename('fullpath'));
mtlb_dir = fileparts(utils_dir);
folder_name = sanitize_name(script_name);
output_dir = fullfile(mtlb_dir,'plots',folder_name);
if ~isfolder(output_dir)
    mkdir(output_dir);
end

saved_files = strings(numel(figure_handles),1);
used_names = containers.Map('KeyType','char','ValueType','double');
for figure_idx = 1:numel(figure_handles)
    fig = figure_handles(figure_idx);
    figure_name = string(fig.Name);
    if strlength(strtrim(figure_name))==0
        figure_name = infer_figure_name(fig);
    end
    base_name = sanitize_name(figure_name);
    key = char(base_name);
    if isKey(used_names,key)
        used_names(key) = used_names(key)+1;
        base_name = base_name+sprintf('_%02d',used_names(key));
    else
        used_names(key) = 1;
    end
    output_file = fullfile(output_dir,char(base_name+'.png'));
    drawnow;
    exportgraphics(fig,output_file,'Resolution',200);
    saved_files(figure_idx) = string(output_file);
end

fprintf('Saved %d figure(s) to %s\n',numel(saved_files),output_dir);
end

function name = infer_figure_name(fig)
axes_handles = findall(fig,'Type','axes');
for idx = numel(axes_handles):-1:1
    candidate = string(axes_handles(idx).Title.String);
    candidate = strjoin(candidate," ");
    if strlength(strtrim(candidate))>0
        name = candidate;
        return
    end
end
name = "figure_"+fig.Number;
end

function clean = sanitize_name(value)
clean = lower(strtrim(string(value)));
clean = replace(clean,["\\_","/","\\"],["_","_","_"]);
clean = regexprep(clean,'[^a-z0-9]+','_');
clean = regexprep(clean,'^_+|_+$','');
if strlength(clean)==0
    clean = "figure";
end
end
