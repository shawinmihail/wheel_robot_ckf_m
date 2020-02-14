clc
clear


q = [100;0;0;0];
q = q / norm(q);

r = [1;-2;-3];

figure
hold on
plot_frame(r, q)
return


q_dual = quatDual(q);
qr = [0;r];

r_rotated1 = quatMultiply(qr, q_dual);
r_rotated1 = quatMultiply(q, r_rotated1);
r_rotated1 = [r_rotated1(2); r_rotated1(3); r_rotated1(4)]

r_rotated2 = quatMultiply(q, qr);
r_rotated2 = quatMultiply(r_rotated2, q_dual);
r_rotated2= [r_rotated2(2); r_rotated2(3); r_rotated2(4)]