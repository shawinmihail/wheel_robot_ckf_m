clc
clear
close all

r0 = [0;0];
phi0 = 0;
R = 1;

ls = pi/2*[5 5 5 5 5 5 5 5]*0.1;

r = r0;
phi = phi0;
k = 1; 
sign_i = 1;
for i = 1:length(ls)
    l = ls(i);
    
    dl = 0.01;
    for s = 0:dl:l
        phi = phi + dl/R;
        r = r + sign_i * dl*[cos(phi); sin(phi)];
        phis(k) = phi;
        rs(:, k) = r;
        k = k+1;
    end
    sign_i = -sign_i;
end

plot(rs(1,:), rs(2,:))