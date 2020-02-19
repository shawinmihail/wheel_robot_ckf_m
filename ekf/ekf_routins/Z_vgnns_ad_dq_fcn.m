function vgnns_q_j = Z_vgnns_ad_dq_fcn(q1,q2,q3,q4,dr1,dr2,dr3,v1,v2,v3,w1,w2,w3)
%Z_VGNNS_AD_DQ_FCN
%    VGNNS_Q_J = Z_VGNNS_AD_DQ_FCN(Q1,Q2,Q3,Q4,DR1,DR2,DR3,V1,V2,V3,W1,W2,W3)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    19-Feb-2020 15:19:47

t2 = abs(v1);
t3 = abs(v2);
t4 = abs(v3);
t5 = dr1.*w2;
t6 = dr2.*w1;
t7 = dr1.*w3;
t8 = dr3.*w1;
t9 = dr2.*w3;
t10 = dr3.*w2;
t11 = t2.^2;
t12 = t3.^2;
t13 = t4.^2;
t14 = -t6;
t15 = -t8;
t16 = -t10;
t17 = t5+t14;
t18 = t7+t15;
t19 = t9+t16;
t32 = t11+t12+t13;
t20 = q1.*t17.*2.0;
t21 = q2.*t17.*2.0;
t22 = q1.*t18.*2.0;
t23 = q3.*t17.*2.0;
t24 = q2.*t18.*2.0;
t25 = q4.*t17.*2.0;
t26 = q1.*t19.*2.0;
t27 = q3.*t18.*2.0;
t28 = q2.*t19.*2.0;
t29 = q4.*t18.*2.0;
t30 = q3.*t19.*2.0;
t31 = q4.*t19.*2.0;
t40 = sqrt(t32);
t33 = -t20;
t34 = -t23;
t35 = -t25;
t36 = -t26;
t37 = -t28;
t38 = -t29;
t39 = -t31;
t41 = q1.*t40.*2.0;
t42 = q2.*t40.*2.0;
t43 = q3.*t40.*2.0;
t44 = q4.*t40.*2.0;
t45 = -t43;
t46 = t21+t22+t39+t44;
t48 = t27+t35+t37+t42;
t49 = t34+t36+t38+t41;
t47 = t24+t30+t33+t45;
vgnns_q_j = reshape([t49,t46,t47,t48,t20-t24-t30+t43,t46,t47,t48,t23+t26+t29-t41,-t21-t22+t31-t44,t49,t48],[3,4]);
