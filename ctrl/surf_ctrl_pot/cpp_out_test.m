clc
clear


obstacle_list = [0.1,0.2,0.3];
r = [0;0;0];
q = [1;0;0;0];
v = 0.5;
rmin = 2.5;
Pi = 0;
Pis = 0;
for i = 1:size(obstacle_list, 1)
    dr = obstacle_list(i,:)' - r;
    ndr = max(norm(dr), 0.05);
    Ce = quatRotate(q, [1;0;0]);
    cos_theta = dr' * Ce / ndr;
    %cos_theta = dr' * [1; 0; 0] / ndr;
    ndrs = - v*cos_theta;
    if (ndr < rmin) && (cos_theta > 0)
        Pi = Pi + 1/ndr;
        Pis = Pis + v*cos_theta/ndr^2;
    end       
end
Pi
Pis
    
    
Pi_mult = 0.5;
lambda = 0.45;
p = r + [-0.3,0.1,0.01]';
Delta = r - p;
delta = norm(Delta);
   
dp = [2;1;0];
ddp = [0;0;0];
Delta_pss = Delta' * ddp;
nvect = cross(Ce, Delta);
crossSign = sign(nvect(3));
norm_ps = norm(dp);
cosPhi = Delta' * Ce / delta;
    


Pi = -Pi;
Pis = -Pis;
mu = lambda;
signX = crossSign;
phi = acos(cosPhi)
u = surf_ctrl_pot_test_1(Delta_pss,Pi,Pi_mult,Pis,delta,mu,norm_ps,phi,signX)
m1 = -abs(delta)/(delta*signX*sin(phi));
s1 = mu*mu*(abs(delta) + Pi*Pi_mult);
s2 = (sin(phi)*sin(phi) + (norm_ps*norm_ps*sin(phi)*sin(phi))/(-norm_ps*norm_ps + Delta_pss))/abs(delta);
s3 = mu * (Pi_mult * Pis + delta * cos(phi) / abs(delta))*2;
u2 = m1*(s1+s2+s3)