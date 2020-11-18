clc
clear
close all
seed = 200;
eps = 1e-2;
g_clr = [0.1 0.9 0.1];

figure
hold on
grid on
ax = gca;
set(gca,'FontSize',14)
hlim = 50;
vlim = 25;
xlim([-hlim, hlim-5]);
ylim([-hlim+5, hlim]);
zlim([-vlim, vlim]);

%% geom
gl = 1;
Tm = 20;
Tc = 90;
Am = 5;

%% main surf
n = 50;
d = 1;
x = -n: d : n;
[X, Y] = meshgrid(x, x);

Z = generate_random_geometry(X, Y, gl, Tm, Tc, Am, seed);
CO = [];
CO(:,:,1) = ones(2*n).*linspace(0.6, 0.8, 2*n); % r
CO(:,:,2) = ones(2*n).*linspace(0.9, 0.9, 2*n); % g
CO(:,:,3) = ones(2*n).*linspace(0.2, 0.2 ,2*n); % b
surf(X,Y,Z,CO, 'EdgeColor', 'None')

%% field
n = 31;
d = 1;
x = -n: d : n;
[X, Y] = meshgrid(x, x);

Z = generate_random_geometry(X, Y, gl, Tm, Tc, Am, seed);
CO = [];
CO(:,:,1) = ones(2*n).*linspace(0.4, 0.4, 2*n); % r
CO(:,:,2) = ones(2*n).*linspace(0.7, 0.8, 2*n); % g
CO(:,:,3) = ones(2*n).*linspace(0.2, 0.2 ,2*n); % b
surf(X,Y,Z,CO, 'EdgeColor', 'None')

% name = ['surf_area__' num2str(n) '_' num2str(d) '.stl']
% surf2stl(name, X, Y, Z);

%% obstcl 1
R = 2;
n = 40;
nR = linspace(0,R,2*n) ;
nT = linspace(0,2*pi,2*n);
[R, T] = meshgrid(nR,nT);
X = 3 * R.*cos(T) + 13; 
Y = 2 * R.*sin(T) + 16;

Z = generate_random_geometry(X, Y, gl, Tm, Tc, Am, seed) + eps;
CO = [];
CO(:,:,1) = ones(2*n).*linspace(0.95, 0.95, 2*n); % r
CO(:,:,2) = ones(2*n).*linspace(0.9, 0.9, 2*n); % g
CO(:,:,3) = ones(2*n).*linspace(0.4, 0.4 ,2*n); % b
surf(X,Y,Z,CO, 'EdgeColor', 'None')

%% obstcl 2
R = 1.8;
n = 40;
nR = linspace(0,R,2*n) ;
nT = linspace(0,2*pi,2*n);
[R, T] = meshgrid(nR,nT);
X = 3 * R.*cos(T-pi/6) - 13; 
Y = 2 * R.*sin(T);

Z = generate_random_geometry(X, Y, gl, Tm, Tc, Am, seed) + eps;
CO = [];
CO(:,:,1) = ones(2*n).*linspace(0.1, 0.1, 2*n); % r
CO(:,:,2) = ones(2*n).*linspace(0.55, 0.55, 2*n); % g
CO(:,:,3) = ones(2*n).*linspace(0.75, 0.85 ,2*n); % b
surf(X,Y,Z,CO, 'EdgeColor', 'None')

%% wr 1
ms1 = 8;
ms2 = 4;

% base
x = -30;
y = 20;
z = surf_fcn(x, y, gl, Tm, Tc, Am, seed) + ms1/10;

% traj
xs = [x, -25, -15  +4, 13, 25, 25, 24, +0, -25, x];
ys = [y, +15, +15, 12, +7, +7, 15, 24, 25, +25, y];
zs = surf_fcn(xs, ys, gl, Tm, Tc, Am, seed) + ms2/10;
traj = [xs; ys; zs];
traj_spline_obj = cscvn(traj);
fnplt(traj_spline_obj, 'linewidth', 0.1, 'r');

b1_plot = plot3(x,y,z, 'ro', 'MarkerSize', ms1, 'MarkerFaceColor', 'r');
% plot3(xs,ys,zs, 'r*', 'MarkerSize', ms2)

%% wr 2
% base
x = -30;
y = 15;
z = surf_fcn(x, y, gl, Tm, Tc, Am, seed) + ms1/10;

% traj
xs = [x, -25, -14, 0, 10, 25, +25, -2, -14, -23, x];
ys = [y, +10, +9., 5, +0, +0, -10, -5, -11, -3., y];
zs = surf_fcn(xs, ys, gl, Tm, Tc, Am, seed) + ms2/10;
traj = [xs; ys; zs];
traj_spline_obj = cscvn(traj);
fnplt(traj_spline_obj, 'linewidth', 0.1, 'g');

plot3(x,y,z, 'go', 'MarkerSize', ms1, 'MarkerFaceColor', 'g')
% plot3(xs,ys,zs, 'g*', 'MarkerSize', ms2)

%% wr 3
% base
x = -30;
y = 10;
z = surf_fcn(x, y, gl, Tm, Tc, Am, seed) + ms1/10;

% traj
xs = [x, -27, -24, -14, ++0, +10, +25, +25, -2., -22, -28, -29, x];
ys = [y, ++0, -11, -19, -13, -15, -20, -25, -25, -25, -11, ++0, y];
zs = surf_fcn(xs, ys, gl, Tm, Tc, Am, seed) + ms2/10;
traj = [xs; ys; zs];
traj_spline_obj = cscvn(traj);
fnplt(traj_spline_obj, 'linewidth', 0.1, 'b');

plot3(x,y,z, 'bo', 'MarkerSize', ms1, 'MarkerFaceColor', 'b')
% plot3(xs,ys,zs, 'b*', 'MarkerSize', ms2)


legend({'surface','field','obstacle #1','obstacle #2',...
    'robot #1 path', 'robot #1 base', ...
    'robot #2 path', 'robot #2 base',...
    'robot #3 path', 'robot #3 base'},...
    'FontSize',14)

%%
function Z = generate_random_geometry(X, Y, gl, Tm, Tc, Am, seed)
    rng(seed)
    Tx = Tm*abs(randn(gl,1))+Tc;
    Ax = Am*randn(gl,1) / gl;
    Px = randn(gl,1);
    Ty = Tm*abs(randn(gl,1))+Tc;
    Ay = Am*randn(gl,1) / gl;
    Py = randn(gl,1);

    Z = 0*X;
    for i=1:gl
        Z = Z + Ax(i) * sin(X*2*pi / Tx(i) + Px(i)) + Ay(i) * sin(Y*2*pi / Ty(i) + Py(i));
    end
end

function z = surf_fcn(x, y, gl, Tm, Tc, Am, seed)
    rng(seed)
    Tx = Tm*abs(randn(gl,1))+Tc;
    Ax = Am*randn(gl,1) / gl;
    Px = randn(gl,1);
    Ty = Tm*abs(randn(gl,1))+Tc;
    Ay = Am*randn(gl,1) / gl;
    Py = randn(gl,1);
    z = x*0;
    for i=1:gl
        z = z + Ax(i) * sin(x*2*pi / Tx(i) + Px(i)) + Ay(i) * sin(y*2*pi / Ty(i) + Py(i));
    end
end

% Z = 0.05 * (X .^2 - Y .^ 2);
% Z = 2*sin(X*2*pi / 25) +  3*sin(Y*2*pi /35);
% name = ['surf_area__' num2str(n) '_' num2str(d) '.stl']
% surf2stl(name, X, Y, Z);
