clc
clear
close all

k = 10;
u_lim = 0.5;

i = 0;
for lambda = 0.0:0.05:1.5
for alpha = 0.25:0.05:0.25
i = i + 1
    


syms z2
% assume(z2, 'real')

eq_minus = -lambda^2 * alpha / k - 2 * lambda * z2 - k + u_lim * sqrt(1-z2^2);

a = [];
b = [];

s = solve(eq_minus, z2);
s1 = vpa(s(1));

if ~isreal(s1)
    s1 = -inf;
end
b = [b; s1];
if length(s) == 2
    s2 = vpa(s(2));
    if ~isreal(s2)
        s2 = inf;
    end
    a = [a; s2];
else
    a = [a; -1];
end

%%
eq_plus = k / ( 1 - alpha) - 2 * lambda * z2 - u_lim * sqrt(1-z2^2);
s = solve(eq_plus, z2);
s1 = vpa(s(1));
if ~isreal(s1)
    s1 = inf;
end
a = [a; s1];
if length(s) == 2
    s2 = vpa(s(2));
    if ~isreal(s2)
        s2 = -inf;
    end
    b = [b; s2];
else
    b = [b; 1];
end


x1(i) = max(a);
x2(i) = min(b);

lambdas(i) = lambda;
alphas(i) = alpha;
dz2(i) = x2(i) - x1(i);
dz1(i) = alpha / k;


end
end

figure
hold on
grid on
xlabel('\lambda')
ylabel('\psi, deg')
set(findall(gcf,'-property','FontSize'),'FontSize',16)
% zlabel('dz2')
plot(lambdas, asin(x1) * 180 / pi, 'g')
plot(lambdas, asin(x2) * 180 / pi, 'r')

% zlabel('dz2')
% plot(lambdas, x1, 'g')
% plot(lambdas, x2, 'r')

% plot(lambdas, dz2, 'k')

% figure
% hold on
% grid on
% % xlabel('lambdas')
% % ylabel('alphas')
% % zlabel('dz2')
% plot(lambdas, dz1, 'k')

% plot3(lambdas, alphas, dz2, 'ko')