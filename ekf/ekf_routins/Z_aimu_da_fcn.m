function imu_a_a_j = Z_aimu_da_fcn(q1,q2,q3,q4)
%Z_AIMU_DA_FCN
%    IMU_A_A_J = Z_AIMU_DA_FCN(Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    06-Mar-2020 14:51:06

t2 = q2.^2;
t3 = q3.^2;
t4 = q4.^2;
t5 = q1.*q2.*2.0;
t6 = q1.*q3.*2.0;
t7 = q1.*q4.*2.0;
t8 = q2.*q3.*2.0;
t9 = q2.*q4.*2.0;
t10 = q3.*q4.*2.0;
t11 = t2.*2.0;
t12 = t3.*2.0;
t13 = t4.*2.0;
t14 = -t11;
t15 = -t12;
t16 = -t13;
imu_a_a_j = reshape([t15+t16+1.0,-t7+t8,t6+t9,t7+t8,t14+t16+1.0,-t5+t10,-t6+t9,t5+t10,t14+t15+1.0],[3,3]);
