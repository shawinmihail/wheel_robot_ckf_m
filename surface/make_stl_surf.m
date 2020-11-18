clc
clear
close all
seed = 205;

figure
hold on
grid on
ax = gca;
set(gca,'FontSize',14)
% hlim = 50;
% vlim = 25;
% xlim([-hlim, hlim-5]);
% ylim([-hlim+5, hlim]);
% zlim([-vlim, vlim]);

%% geom
gl = 2; % number of garmonic
Tm = 20; % rms of random part of period
Tc = 160; % constant part of period
Am = 5; % rms of random part of magnitude

%% main surf
n = 120; % size
d = 1; % resolution
x = -n: d : n;
[X, Y] = meshgrid(x, x);

Z = generate_random_geometry(X, Y, gl, Tm, Tc, Am, seed);
CO = [];
CO(:,:,1) = ones(2*n).*linspace(0.6, 0.8, 2*n); % r
CO(:,:,2) = ones(2*n).*linspace(0.9, 0.9, 2*n); % g
CO(:,:,3) = ones(2*n).*linspace(0.2, 0.2 ,2*n); % b
surf(X,Y,Z,CO, 'EdgeColor', 'None')

%% stl
% name = 'surface/generated/surf.stl';
% surf2stl(name, X, Y, Z);

%% gen
% m
syms x
syms y
[zfcn, grad_zfcn] = surf_fcn_generator(x, y, gl, Tm, Tc, Am, seed);
matlabFunction([zfcn; grad_zfcn],'File','surface/generated/surf_fcn');
% c
cfg = coder.config('dll');
cfg.TargetLang = 'C++'; 
codegen -config cfg surf_fcn -args {0, 0}
copyfile codegen/dll/surf_fcn/surf_fcn.so surface/generated/surf_fcn.so
copyfile codegen/dll/surf_fcn/surf_fcn.h surface/generated/surf_fcn.h
copyfile codegen/dll/surf_fcn/rtwtypes.h surface/generated/rtwtypes.h
copyfile codegen/dll/surf_fcn/surf_fcn_spec.h surface/generated/surf_fcn_spec.h
copyfile codegen/dll/surf_fcn/surf_fcn_terminate.h surface/generated/surf_fcn_terminate.h
copyfile codegen/dll/surf_fcn/surf_fcn_types.h surface/generated/surf_fcn_types.h


% other
x0 = 40;
y0 = 20;
out = surf_fcn(x0,y0);
z0 = out(1);
g0 = out(2:4);
g0 = g0 * 20;
plot3(x0, y0, z0, 'ko')
plot3([x0 x0 + g0(1)], [y0 y0 + g0(2)], [z0 z0 + g0(3)], 'r')

%% functions
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

function z = surf_fcn_m(x, y, gl, Tm, Tc, Am, seed)
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

function [zfcn, grad_zfcn] = surf_fcn_generator(x, y, gl, Tm, Tc, Am, seed)
    rng(seed)
    Tx = Tm*abs(randn(gl,1))+Tc;
    Ax = Am*randn(gl,1) / gl;
    Px = randn(gl,1);
    Ty = Tm*abs(randn(gl,1))+Tc;
    Ay = Am*randn(gl,1) / gl;
    Py = randn(gl,1);
    zfcn = @(x,y)x*0;
    for i=1:gl
        zfcn = zfcn + Ax(i) * sin(x*2*pi / Tx(i) + Px(i)) + Ay(i) * sin(y*2*pi / Ty(i) + Py(i));
    end
    dx = diff(zfcn, x);
    dy = diff(zfcn, y);
    grad_zfcn = [dx; dy; 1];
end

