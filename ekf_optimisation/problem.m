function QUALITY = problem(x)

%%
q = [100;10;5;0];
q = q / norm(q);
initial_est_state = [[0;0;0];[0;0;0];[0;0;0];q;[0;0;0]];

a_imu_xz = x(1);
a_gnns_x = x(2);
a_gnns_z = x(3);
w_imu_xz = x(4);
r_gnns_x = x(5);
r_gnns_z = x(6);
v_gnns_x = x(7);
v_gnns_z = x(8);
u_gnns_x = x(9);
u_gnns_z = x(10);

R_a_imu = diag(10 .^ ([a_imu_xz; a_imu_xz; a_imu_xz]));
R_a_gnns = diag(10 .^ ([a_gnns_x; a_gnns_x; a_gnns_z]));
R_w_imu = diag(10 .^ ([w_imu_xz; w_imu_xz; w_imu_xz]));
R_rv_gnns = diag(10 .^ ([[r_gnns_x; r_gnns_x; r_gnns_z]; [v_gnns_x; v_gnns_x; v_gnns_z]]));
R_u_gnns = diag(10 .^ ([u_gnns_x; u_gnns_x; u_gnns_z]));


qr = x(11);
qv = x(12);
qax = x(13);
qaz = x(14);
qq0 = x(15);
qqx = x(16);
qqz = x(17);
qwx = x(18);
qwz = x(19);

Q = diag(10 .^ ([[qr; qr; qr]; [qv; qv; qv]; [qax; qax; qaz]; [qq0; qqx; qqx; qqz]; [qwx; qwx; qwz]]));
P0 = 50*Q;
initial_sqrtP = chol(P0,'lower');
%%

ekf_rosbag_play

QUALITY = 0;
cost
QUALITY

end  