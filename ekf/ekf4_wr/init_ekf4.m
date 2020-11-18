q = [100;10;5;0];
q = q / norm(q);
initial_est_state = [[0;0;0];[0;0;0]; q];

R_a_gnns = diag(1e-0*[1; 1; 1]);
R_rv_gnns = diag([1e-2*[1; 1; 1]; 1e-4*[1; 1; 1]]);
R_u_gnns = diag(1e-4*[1; 1; 1]);
R_q2_gnns = diag(1e-6*[1; 1; 1; 1; 1; 1]);

Q = diag([1e-1*[1; 1; 1]; 1e-1*[1; 1; 1]; 1e-1*[1; 1; 1]]);
%% _-----_------------_--------

% load('ekf_optimisation/out/x')
% a_gnns_x = x(1);
% a_gnns_z = x(2);
% r_gnns_x = x(3);
% r_gnns_z = x(4);
% v_gnns_x = x(5);
% v_gnns_z = x(6);
% u_gnns_x = x(7);
% u_gnns_z = x(8);
% 
% R_a_gnns = diag(10 .^ ([a_gnns_x; a_gnns_x; a_gnns_z]));
% R_rv_gnns = diag(10 .^ ([[r_gnns_x; r_gnns_x; r_gnns_z]; [v_gnns_x; v_gnns_x; v_gnns_z]]));
% R_u_gnns = diag(10 .^ ([u_gnns_x; u_gnns_x; u_gnns_z]));
% 
% 
% qr = x(9);
% qv = x(10);
% qqx = x(11);
% qqz = x(12);
% 
% Q = diag(10 .^ ([[qr; qr; qr]; [qv; qv; qv]; [qqx; qqx; qqz]]));

%% _-----_------------_--------
P0 = 50*Q;


