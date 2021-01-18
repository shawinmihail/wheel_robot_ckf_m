function u = u_template2(Pi_mult, ro, k, z1, psi, r, theta, rmin)


eps = 1e-6;

z2 = sin(psi);
mu = 0.5;
r1 = r(1);
r2 = r(2);
ro1 = ro(1);
ro2 = ro(2);
ev1 = cos(theta);
ev2 = sin(theta);
evx1 = -sin(theta);
evx2 = cos(theta);
ndr = sqrt((ro1-r1)*(ro1-r1) + (ro2-r2)*(ro2-r2));

if (z1 >= 0 && z1 < eps)
    z1 = eps;
elseif (z1 < 0 && z1 > -eps)
    z1 = -eps;
end
cosTheta = 0;
if (ndr >= 0 && ndr < eps)
ndr = eps;
elseif (ndr < 0 && ndr > -eps)
    ndr = -eps;
else
    cosTheta = ev1*(ro1 - r1)/ndr + ev2*(ro2 - r2)/ndr;
end    
sinThetaSq = 1.0 - cosTheta*cosTheta;
u = u_obs_pot_generated2(Pi_mult, ev1,ev2,evx1,evx2,k,mu,ndr,psi,r1,r2,rmin,ro1,ro2,sinThetaSq,z1,z2);

end

