function vgnns_v_j = Z_vgnns_ad_dv_fcn(v1,v2,v3,q1,q2,q3,q4)
%Z_VGNNS_AD_DV_FCN
%    VGNNS_V_J = Z_VGNNS_AD_DV_FCN(V1,V2,V3,Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    05-Mar-2020 18:33:49

t2 = abs(v1);
t3 = abs(v2);
t4 = abs(v3);
t5 = sign(v1);
t6 = sign(v2);
t7 = sign(v3);
t8 = q3.^2;
t9 = q4.^2;
t13 = q1.*q3.*2.0;
t14 = q1.*q4.*2.0;
t15 = q2.*q3.*2.0;
t16 = q2.*q4.*2.0;
t10 = t2.^2;
t11 = t3.^2;
t12 = t4.^2;
t17 = t8.*2.0;
t18 = t9.*2.0;
t19 = -t16;
t20 = t14+t15;
t21 = t13+t19;
t22 = t10+t11+t12;
t23 = t17+t18-1.0;
t24 = 1.0./sqrt(t22);
vgnns_v_j = reshape([-t2.*t5.*t23.*t24,t2.*t5.*t20.*t24,-t2.*t5.*t21.*t24,-t3.*t6.*t23.*t24,t3.*t6.*t20.*t24,-t3.*t6.*t21.*t24,-t4.*t7.*t23.*t24,t4.*t7.*t20.*t24,-t4.*t7.*t21.*t24],[3,3]);
