clc
clear

tic
n = 20;
m = n+1;
for i = 1: 100000
    A1 = randn(n,m);
    A2 = randn(m,n);
    A3 = A1*A2;
end
toc