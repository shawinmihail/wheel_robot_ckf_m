clc
clear 
close all

figure
hold on
axis equal
xlim([-20 20])
ylim([-20 20])

% rng(150)

r = [0; 0];
v = [1; 0];

ps = [];
for i = 1:20
    p = 19*randn(2,1) + [6;0];
    plot(p(1), p(2), 'r*')
    ps = [ps p];
end

for k = 1:30

pot_list = [];
for i = 1:length(ps(1,:))
    p = ps(:, i);
    
    dr = p - r;
    n_dr = norm(dr);
    dir_dr = dr/norm(dr);
    
    dot1 = v(1)*dr(1) + v(2)*dr(2);
    det1 = v(1)*dr(2) - p(1)*dr(2);
    angle = atan2(det1, dot1);
    
    if dot1 > 0 || 1
        pot = 10 * dir_dr * abs(dot1) / n_dr / n_dr;
        pot_list = [pot_list pot];  
    else
        pot_list = [pot_list [0;0]];
    end
end

sum_pot = sum(pot_list, 2);
n_sum_pot = norm(sum_pot);
plot([r(1) r(1) + sum_pot(1)], [r(2) r(2) + sum_pot(2)], 'b')

nv_sum_pot = [1; -sum_pot(1) / sum_pot(2)];
nv_sum_pot = nv_sum_pot / norm(nv_sum_pot);
plot([r(1) r(1) + nv_sum_pot(1)], [r(2) r(2) + nv_sum_pot(2)], 'k')
plot([r(1) r(1) - nv_sum_pot(1)], [r(2) r(2) - nv_sum_pot(2)], 'k')

dot_p = dot(v,nv_sum_pot);

nv_sum_pot = sign(dot_p)*nv_sum_pot;
plot([r(1) r(1) + nv_sum_pot(1)], [r(2) r(2) + nv_sum_pot(2)], 'm')

shift_dir = [0;0];
if dot_p > 0
    shift_dir = nv_sum_pot;
else
    shift_dir = -nv_sum_pot;
end
    

plot(r(1), r(2), 'ko')
r = r + 0.5*v + n_sum_pot*shift_dir/10;
end
