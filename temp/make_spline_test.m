clc
clear
close all

t = 0: 1 : 30;
x = 10 * sin (t*2*pi/ 30);
y = 15 * cos (t*2*pi/ 30) - 15;
z = 2*sin(x*2*pi / 25) +  3*sin(y*2*pi /35);

figure
hold on
grid on
plot3(x,y,z, 'ko')

for i = 1:length(t)-3
    ps = [];
    for k = 0:3
        ps = [ps; x(i+k) y(i+k) z(i+k)];
    end
    M = 1/6 * [-1 3 -3 1; 3 -6 3 0; -3 0 3 0; 1 4 1 0]*ps;
    
    for a = 0:0.1:1
        xa = a^3 * M(1,1) + a^2 * M(2,1) + a^1 * M(3,1) + a^0 * M(4,1);
        ya = a^3 * M(1,2) + a^2 * M(2,2) + a^1 * M(3,2) + a^0 * M(4,2);
        za = a^3 * M(1,3) + a^2 * M(2,3) + a^1 * M(3,3) + a^0 * M(4,3);
        plot3(xa,ya,za, 'ro')
    end

end




    
   
