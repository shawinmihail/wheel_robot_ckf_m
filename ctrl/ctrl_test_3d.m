clc
clear
close all


%% initial values and constantspoten
%rand0
seed = 565;
% rng(seed);

% loop
dt = 1e-2;
N = 10000;

% surf
[surf_fcn, grad_surf] = plane_surf();
surf_fcn = @(x,y)(surf_fcn(x,y) - 2.5);
% [surf_fcn, grad_surf] = custom_sur && (dot1 > 0)f();

% initial
initial_x = -9;
initial_y = 9;
initial_z = surf_fcn(initial_x, initial_y);
initial_r = [initial_x;initial_y;initial_z];
initial_v = [0;0;0];
initial_yaw = pi;
initial_q = calc_q_full(grad_surf, initial_r,initial_yaw);
initial_w = [0;0;0];
initial_state = [initial_r; initial_v; initial_q];
initial_ctrl = [0; 0];



%% preproc
curr_state = initial_state;
curr_yaw = initial_yaw;
curr_w = initial_w;
curr_ctrl = initial_ctrl;

act_states = [];
timeline = [];
%% sim
t = 0;

%  route
tb = readtable('car2.csv');
set = tb{1:200,1:end};
set = set';
[splines] = M_spline_from_set(set); 

%
spline_index = 1;
error_cntr = 0;

obs0 = [-18 16 -2.5];
obstacle_list = [];
% for i = 1:15
%     obstacle_list = [obstacle_list; obs0 + [5*randn(1,2) obs0(3)]];
% end
% obstacle_list = [obs0 - [0 0.5 0]; obs0 + [0 0.0 0]];
% obstacle_list = [obs0 - [0 2 0]];
% obstacle_list = [-10 14 0];
integrator_res = 0;
for i = 1:N
    
    i    
    
    %% actuators dyn modeling
%     next_ctrl = [3 ; 0.1];
    y = curr_state(1:3);
    q = curr_state(7:10);
    C = quat2matrix(q);
    v = 1;

    [u, v, sstar, pstar, DELTA, spline_index, error_cntr, delta_h, dot_p]  = calculate_ctrl_3d_grad_pot_avoidance(y, v, C, splines, spline_index, error_cntr, obstacle_list);
    
    if (abs(delta_h) < 0.25 && dot_p > 0.96)
        integrator_res = integrator(delta_h, dt, 2, integrator_res);
    end

    du_luft = (5+0.5*randn) * sin(t*2*pi + 1*randn*2*pi/20) * pi / 180;
    luft(i) = du_luft;
    
    
    usc(i) = u;
    u = u + du_luft;
    
    ir(i) = integrator_res;
    dy(i) = delta_h;
    dotp(i) = dot_p;
 
    error_cntrs(i) = error_cntr;
    spline_indexs(i) = spline_index;
    us(:, i) = u;
    sstars(: ,i) = sstar;
    pstars(:, i) = pstar;
    deltas(:, i) = C' * DELTA;
    deltas_norm(:, i) = norm(DELTA);
    ts(i) = t;
    
    next_ctrl = [v ; u];
 
    %% wheel robot state evolution
    % state = [r v q]
    [next_state, next_yaw] = calculate_next_state(curr_state, curr_yaw, next_ctrl, grad_surf, dt);    
    [next_a, next_w] = calculate_acc_rotrate(curr_state, next_state, dt);
    next_w_dot = (next_w - curr_w) / dt;
    
    %% full state = [r v a q w]
    full_state_curr = [next_state(1:6);next_a;next_state(7:10);next_w; next_w_dot];

    %% sim next step
    curr_ctrl = next_ctrl;
    curr_state = next_state;
    curr_yaw = next_yaw;
    curr_w = next_w;
    t = t + dt;
    
    %% save
    act_states(:, i) = full_state_curr;
    timeline(i) = t;
end

% figure
% hold on
% title('delta')
% plot(deltas_norm)


% figure
% hold on
% title('us')
% plot(timeline, us)
% 
% figure
% hold on
% title('sstars')
% plot(timeline, sstars)
% 
% figure
% hold on
% plot(timeline, deltas(1, :), 'r')
% plot(timeline, deltas(2, :), 'g')
% plot(timeline, deltas(3, :), 'b')
% 
% figure
% hold on
% fs = 20;
% set(gca,'FontSize',fs)
% title('\delta(t)')
% xlabel('t, s')
% ylabel('\delta, m')
% plot(timeline, deltas_norm, 'r')

figure
hold on
grid on
plot(ts, us, 'k')
plot(ts, usc, 'r')
ret

splines_s = size(splines);
for i = 1:splines_s(3)

spline = splines(:,:,i);

for a = 0:0.1:1
    spline;
    point = spline * [1; a; a^2; a^3];
    plot3(point(1), point(2), point(3), 'r.')
end

end

% plot3(act_states(1,1),act_states(2,1),act_states(3,1), 'r*');
plot3(act_states(1,:),act_states(2,:),act_states(3,:), 'k--');
plot3(obstacle_list(:,1), obstacle_list(:,2), obstacle_list(:,3), '*','Color','g','MarkerSize',10);
