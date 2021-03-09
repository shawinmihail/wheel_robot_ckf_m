clc
clear
close all
% rng(201)
eps = 1e-3;


% oc grid initial
oc_grid_size = 6;
oc_grid_resolution = 1;
oc_grid_dim = ceil(oc_grid_size/oc_grid_resolution); 
oc_grid_size = oc_grid_dim * oc_grid_resolution;
oc_grid_matrix = zeros(oc_grid_dim, oc_grid_dim);
% oc_grid_matrix = randn(oc_grid_dim, oc_grid_dim);

% segment

p1s = [0 0 0; 0 0 0];
p2s = [1 2 -1.75; 2 1 2.25];
oc_grid_matrix = refresh_grid(p1s, p2s, oc_grid_matrix, oc_grid_size, oc_grid_resolution)
