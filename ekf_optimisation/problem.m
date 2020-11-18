function QUALITY = problem(x)

%%
q = [100;10;5;0];
q = q / norm(q);
initial_est_state = [[0;0;0];[0;0;0]; q];


a_gnns_x = x(1);
a_gnns_z = x(2);
r_gnns_x = x(3);
r_gnns_z = x(4);
v_gnns_x = x(5);
v_gnns_z = x(6);
u_gnns_x = x(7);
u_gnns_z = x(8);
q_gnns_x = x(9);
q_gnns_z = x(10);

R_a_gnns = diag(10 .^ ([a_gnns_x; a_gnns_x; a_gnns_z]));
R_rv_gnns = diag(10 .^ ([[r_gnns_x; r_gnns_x; r_gnns_z]; [v_gnns_x; v_gnns_x; v_gnns_z]]));
R_u_gnns = diag(10 .^ ([u_gnns_x; u_gnns_x; u_gnns_z]));
R_q2_gnns = diag(10 .^ [q_gnns_x; q_gnns_x; q_gnns_z; q_gnns_x; q_gnns_x; q_gnns_z]);

qr = x(11);
qv = x(12);
qqx = x(13);
qqz = x(14);

Q = diag(10 .^ ([[qr; qr; qr]; [qv; qv; qv]; [qqx; qqx; qqz]]));
P0 = 50*Q;
%%

ekf_rosbag_play_ekf4

QUALITY = 0;
cost


end  