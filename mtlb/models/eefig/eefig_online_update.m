function status = eefig_online_update(X_measured, U_measured, X_next_measured)
%EEFIG_ONLINE_UPDATE Adapt after the next measured state becomes available.
% Call eefig(...) first to predict, score that prediction, and only then call
% this function. This function intentionally cannot be called implicitly by
% eefig(), keeping frozen and adaptive evaluations separate.

validateattributes(X_measured, {'numeric'}, ...
    {'vector','numel',6,'finite'});
validateattributes(U_measured, {'numeric'}, ...
    {'vector','numel',2,'finite'});
validateattributes(X_next_measured, {'numeric'}, ...
    {'vector','numel',6,'finite'});

x_lat = X_measured([2, 4]);
u_lat = [U_measured(1); X_measured(5)];
x_lat_next = X_next_measured([2, 4]);
status = eefig_runtime('update', x_lat, u_lat, x_lat_next);
end
