clear
clc
rng(1)

x = 1:0.1:10;
y = (5. + randn(1,length(x))) .* sin(x + pi/10*randn(1,length(x)));

plot(x,y)

