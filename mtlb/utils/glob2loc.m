function [x_loc, y_loc] = glob2loc(x_glob, y_glob, psi_glob)
%GLOB2LOC Rotate a global-frame vector into a heading-aligned local frame.
% Inputs may be scalars or arrays with compatible sizes. Translation is not
% applied; x_glob and y_glob must already be relative to the local origin.

validateattributes(x_glob, {'numeric'}, {'real', 'finite'});
validateattributes(y_glob, {'numeric'}, {'real', 'finite'});
validateattributes(psi_glob, {'numeric'}, {'real', 'finite'});

x_loc = cos(psi_glob) .* x_glob + sin(psi_glob) .* y_glob;
y_loc = -sin(psi_glob) .* x_glob + cos(psi_glob) .* y_glob;
end
