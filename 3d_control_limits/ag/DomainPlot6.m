clear all;
hold off;
%% System
lambda = 1.5;%3;%1.5;%3; %2.5;
c = [lambda*lambda; 2*lambda];

A = [0, 1;
     0, 0];

b = [0; -1];

% umax = 0.2;
umax = 0.5;%0.7;%0.5; %tan(20*pi/180)/0.7; 
%%
z1_max = 2; 
z1_step = z1_max/100;
z2_max = 2;
z2_step = z2_max/100;
%% LMI P (Lyr'e) 
V1 = 1.5;%1;%1.2;%1.4;%1.77;
V2 = 1;

mu = 0.1*lambda;

a = 1.2;%0.45;%1.6;%1.28;%1.15;
sigma0 = 2;

sigma2 = sigma0;
gamma0 = (1+a*a);
beta0 = umax/sigma0;
beta2 = umax/sigma2;

alpha1 = 0.2;
alpha2 = a;

cvx_begin sdp
variable P(2,2) symmetric
variables tau1 tau2 tau3
variable theta
e05 = -0.5*[0;1];
e2c = 0.5*(c*[0;1]'+[0;1]*c');
u2  = umax*umax;
u2g2 = umax*umax*gamma0*gamma0;
beta1 = beta0 + 1;
%% theta >= 0 
Pmax = P + 0*[0, 0; 0, 1.5*theta*umax*umax];
%% theta <= 0 
% Pmax = P+0.5*theta*c*c'; 
%%
minimize( trace(Pmax) )
subject to
   tau1 >= 0
   tau2 >= 0
   tau3 >= 0
%% 
   theta >= 0
   Pv = [P,c*theta/2;c'*theta/2,-0.5*theta]; %Pv = [P+c*c'*theta/2,[0;0];[0,0],0]; //more conservative
   
   2*mu*Pv + [P*A+A'*P - tau1*beta0*c*c' + tau2*e2c - 3*theta*u2*e2c,   P*b + 0.5*theta*A'*c + 0.5*tau1*beta1*c  + tau2*e05 - 3*theta*u2*e05;   b'*P + 0.5*theta*c'*A + 0.5*tau1*beta1*c' + tau2*e05' - 3*theta*u2*e05',   theta*c'*b - tau1] <= 0
   2*mu*Pv + [P*A+A'*P - tau1*beta0*c*c' - tau3*e2c - 3*theta*u2g2*e2c, P*b + 0.5*theta*A'*c + 0.5*tau1*beta1*c  - tau3*e05 - 3*theta*u2g2*e05; b'*P + 0.5*theta*c'*A + 0.5*tau1*beta1*c' - tau3*e05' - 3*theta*u2g2*e05', theta*c'*b - tau1] <= 0
   [Pmax, c; c', ( sigma0 )^2] >= 0
%% 
%    theta <= 0
%    Pv = [P,[0;0];[0,0],0];
%    
%    2*mu*Pv + [P*A+A'*P - tau1*beta0*c*c' + tau2*e2c - 3*theta*u2g2*e2c, P*b + 0.5*theta*A'*c + 0.5*tau1*beta1*c  + tau2*e05 - 3*theta*u2g2*e05; b'*P + 0.5*theta*c'*A + 0.5*tau1*beta1*c' + tau2*e05' - 3*theta*u2g2*e05',   theta*c'*b - tau1] <= 0
%    2*mu*Pv + [P*A+A'*P - tau1*beta0*c*c' - tau3*e2c - 3*theta*u2*e2c,   P*b + 0.5*theta*A'*c + 0.5*tau1*beta1*c  - tau3*e05 - 3*theta*u2*e05;   b'*P + 0.5*theta*c'*A + 0.5*tau1*beta1*c' - tau3*e05' - 3*theta*u2*e05', theta*c'*b - tau1] <= 0
%    [Pmax, c; c', ( sigma0 )^2] >= 0
%%
%    Pmax >= [1/(alpha1^2), 0; 0 ,0];
%    Pmax >= [0, 0; 0 ,1/(alpha2^2)]; 
cvx_end
%% S-procedure
cvx_begin sdp
variable P0(2,2) symmetric
variables tau1
Pv = [P0,[0;0];[0,0],0];
minimize( trace(P0) )
subject to
   2*mu*Pv + [P0*A+A'*P0 - tau1*beta2*c*c', P0*b + 0.5*tau1*beta2*c + 0.5*tau1*c; b'*P0 + 0.5*tau1*c'*beta2 + 0.5*tau1*c', - tau1] <= 0
   [P0, c; c', ( umax/beta2 )^2] >= 0
   tau1 >= 0;
%    P0 >= [1/(alpha1^2), 0; 0 ,0];
%    P0 >= [0, 0; 0 ,1/(alpha2^2)]; 
cvx_end

eigvalP0 = eig(P0-eye(2));
%% w/o S-procedure
% beta = beta0;
% % mu = 0;
% Ab = [0, 1;
%      -beta*lambda*lambda, -2*beta*lambda];
% A = [0, 1;
%      -lambda*lambda, -2*lambda];
%  
% cvx_begin sdp
% variable P0(2,2) symmetric
% 
% minimize( trace(P0) )
% subject to
%     P0*A + A'*P0 + 2*mu*P0 <= 0
%     P0*Ab + Ab'*P0 + 2*mu*P0 <= 0
%     [P0, c; c', (umax/beta)^2] > 0
% % P0 >= eye(2)
%     P0 >= [1/(alpha1^2), 0; 0 ,0];
% cvx_end
% 
% eigvalP0 = eig(P0-eye(2));
%%
in = [];
in2 = [];
in3 = [];
in4 = [];
in5 = [];

% for z1 = -z1_max:z1_step:z1_max
%     for z2 = -z2_max:z2_step:z2_max
%         z = [z1;z2];
%         sigma = c'*z;
%         if sigma >= umax*(1+z2*z2)^1.5   
%             addval = theta*(umax*(1+z2*z2)^1.5)*(sigma-0.5*umax*(1+z2*z2)^1.5);
%         elseif sigma <= -umax*(1+z2*z2)^1.5   
%             addval = theta*(umax*(1+z2*z2)^1.5)*(-sigma-0.5*umax*(1+z2*z2)^1.5);
%         else %if abs(sigma) < umax*(1+z2*z2)^1.5
%             addval = theta*(sigma^2)/2;
%         end
%         
%         val = z'*P0*z;
%         val2 = z'*P*z + addval;
%         val3 = z'*(Pmax)*z;
% %         val3 = z'*(P+0.5*theta*c*c')*z;
%         
%         if(val <= V2)
%             in = [in; z1,z2];
%         end
%         if(val2 <= V1) 
%             in2 = [in2; z1,z2];
%         end
%         if(val3 <= V1) 
%             in3 = [in3; z1,z2];
%         end
%         if((val2 - V1)*(val2 - V1 + 3*max(z1_step, z2_step)) <= 0)
%             in4 = [in4; z1,z2];
%         end
%         if abs(sigma) >= umax*(1+z2*z2)^1.5
%              in5 = [in5; z1,z2];
%         end
%     end
% end


[z1,z2]=meshgrid(-z1_max:z1_step:z1_max);
val = zeros(size(z1,1),size(z2,1));
for i = 1:size(z1,1)
    for j = 1:size(z1,2)
        val(i,j) = Vfun(z1(i,j), z2(i,j), theta, umax, c, P);
        
        z = [z1(i,j);z2(i,j)];
        sigma = c'*z;
        val2(i,j) = abs(sigma) - umax*(1+z2(i,j)*z2(i,j))^1.5;
    end
end

% h = zeros(1,2);
% plot(in3(:,1),in3(:,2), '.', 'Color', [.5 .5 .5]); hold on;

% h(1) = plot(in(:,1),in(:,2), 'b.', 'DisplayName', '$z^{T}P_{0}z\leq 1$'); hold on;
% h(2) = plot(in2(:,1),in2(:,2), 'r.', 'DisplayName', '$z^{T}Pz+\theta\int\Phi d\sigma\leq 1$'); hold on;
%%
t = 0:pi/30:2*pi;
[V,D] = eig(P0);
x1 = sqrt(V2/D(1))*cos(t);
y1 = sqrt(V2/D(4))*sin(t);
Y = [x1;y1];
X = Y'*V;
plot(X(:,1),X(:,2),'b-', 'LineWidth', 3); hold on;
%%
M = contour(z1,z2,val,[V1-0.001:0.001:V1], 'r-', 'LineWidth', 3);
%%
% t = 0:pi/30:2*pi;
% [V,D] = eig(Pmax);
% x1 = sqrt(V1/D(1))*cos(t);
% y1 = sqrt(V1/D(4))*sin(t);
% Y = [x1;y1];
% X = Y'*V;
% plot(X(:,1),X(:,2),'-', 'LineWidth', 2, 'Color', [.5 .5 .5]);
%%
contour(z1,z2,val2,[0-0.001:0.001:0], 'm--', 'LineWidth', 2);
%%
grid minor; daspect([1;1;1]);  xlim([-z1_max,z1_max]); ylim([-z2_max,z2_max]);


line([-z1_max, z1_max],[a, a],'Color','black','LineStyle','--', 'LineWidth', 2)
line([-z1_max, z1_max],[-a, -a],'Color','black','LineStyle','--', 'LineWidth', 2)

% line([-alpha1, -alpha1],[-z2_max, z2_max], 'Color','black','LineStyle','--', 'LineWidth', 2)
% line([alpha1, alpha1],[-z2_max, z2_max],'Color','black','LineStyle','--', 'LineWidth', 2)

%z1 + 2*z2 = sigma0;
xplot = -z1_max:z1_step:z1_max;
yplot1 = (sigma0-lambda*lambda*xplot)/2/lambda;
yplot2 = (-sigma0-lambda*lambda*xplot)/2/lambda;
plot(xplot, yplot1, 'black--', 'LineWidth', 2);
plot(xplot, yplot2, 'black--', 'LineWidth', 2);

xlabel('z_1','FontSize',16);
ylabel('z_2','FontSize',16);
set(gca,'FontSize',16)
% title(['$\overline{u} = ',num2str(umax),',V = ',num2str(V1),',a = ',num2str(a),',\sigma_{0} = ',num2str(sigma0),', \lambda = ',num2str(lambda),', \mu = ',num2str(round(mu,2)),', \theta = ',num2str(round(theta,2)),'$'],'interpreter','latex','FontSize',16)

% hl = legend(h(1:2)); %legend('show');
% set(hl, 'Interpreter','latex')