function [splines] = M_spline_from_set(set)

% additional for use first point
d = set(:,2) - set(:,1);
set0 = set(:,1) - d;
set = [set0 set];

% additional for use two last point
d = set(:,end) - set(:,end-1);
set_end = set(:,end) + d;
set = [set set_end];
set_end = set(:,end) + d;
set = [set set_end];

% build spline
M = 1/6 * [1 -3 3 -1; 4 0 -6 3; 1 3 3 -3; 0 0 0 1];
for i = 2:length(set)-3
ri = [set(:, i-1) set(:, i) set(:, i+1) set(:, i+2)];
A = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
spline = ri * M * A;
splines(:, :, i-1) = spline;
end

