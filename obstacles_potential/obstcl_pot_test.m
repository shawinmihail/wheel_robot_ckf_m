clear
close all

global U_
global v_
global b
global bet
global m_
global P_
global k_
global k2
global alp
global mu
global NO
global NO1
global NO2
global v0
global W_
global X_O
global rmin
global aprev
global delta0


step = 0.1;
%StepX = 3;
t0 = 0;
t1 = 30;
x_0 = 0.;
y_0 = 3;
z_0 = 0.;
thet_0 = pi/4;

k_ = 2;
k2 = 0;%k_;
m_ = 5.;

aprev = 0;

mu = 2.;

b = 1;

bet = 0.05;%3
rmin = 9;
alp = 1.2;%0
%rmin = 1;
%alp = 3;%0

delta0 = 0.2;

Vmin = log(rmin*rmin - delta0*delta0);
% rand('normal');
%X_O = [
%  [10; 0.5; 0] [10.1; 0.5; 0] [10.2; 0.5; 0] [10.3; 0.5; 0] [10.4; 0.5; 0] [10.5; 0.5; 0] [10.6; 0.5; 0] [10.7; 0.5; 0] [10.8; 0.5; 0] [10.9; 0.5; 0] [11; 0.5; 0] [11.1; 0.5; 0]...
%  [11.1; 0.4; 0] [11.1; 0.3; 0] [11.1; 0.2; 0] [11.1; 0.1; 0] [11.1; 0.; 0] [11.1; -0.1; 0] [11.1; -0.2; 0] [11.1; -0.3; 0] [11.1; -0.4; 0] [11.1; -0.5; 0] ...
%  [11.2; -0.5; 0] [11.3; -0.5; 0] [11.4; -0.5; 0] [11.5; -0.5; 0] [11.6; -0.5; 0] [11.7; -0.5; 0] [11.8; -0.5; 0] [11.9; -0.5; 0] [12; -0.5; 0]  [12.1; -0.5; 0]  [12.2; -0.5; 0]  [12.3; -0.5; 0] [12.4; -0.5; 0] [12.5; -0.5; 0] [12.6; -0.5; 0] [12.7; -0.5; 0] [12.8; -0.5; 0] [12.9; -0.5; 0] [13; -0.5; 0] [13.1; -0.5; 0] [13.2; -0.5; 0] [13.3; -0.5; 0] [13.4; -0.5; 0] [13.5; -0.5; 0] [13.6; -0.5; 0] [13.7; -0.5; 0] [13.8; -0.5; 0] [13.9; -0.5; 0] [14; -0.5; 0] [14.1; -0.5; 0] [14.2; -0.5; 0] [14.3; -0.5; 0] ...
%];
%X_O = [
%  [10; 0.5; 0] [10.1; 0.5; 0] [10.2; 0.5; 0] [10.3; 0.5; 0] [10.4; 0.5; 0] [10.5; 0.5; 0] [10.6; 0.5; 0] [10.7; 0.5; 0] [10.8; 0.5; 0] [10.9; 0.5; 0] [11; 0.5; 0] ...
%  [11.1; 0.4; 0] [11.2; 0.3; 0] [11.3; 0.2; 0] [11.4; 0.1; 0] [11.5; 0.; 0] [11.6; -0.1; 0] [11.7; -0.2; 0] [11.8; -0.3; 0] [11.9; -0.4; 0] [12; -0.5; 0] ...
%  [12.1; -0.5; 0] [12.2; -0.5; 0] [12.3; -0.5; 0] [12.4; -0.5; 0] [12.5; -0.5; 0] [12.6; -0.5; 0] [12.7; -0.5; 0] [12.8; -0.5; 0] [12.9; -0.5; 0]  [13.0; -0.5; 0]  [13.1; -0.5; 0]  [13.2; -0.5; 0] [13.3; -0.5; 0] [13.4; -0.5; 0] [13.5; -0.5; 0] [13.6; -0.5; 0] [13.7; -0.5; 0] [13.8; -0.5; 0] [13.9; -0.5; 0] [14.; -0.5; 0] [14.1; -0.5; 0] [14.2; -0.5; 0] [14.3; -0.5; 0] [14.4; -0.5; 0] [14.5; -0.5; 0] [14.6; -0.5; 0] [14.7; -0.5; 0] [14.8; -0.5; 0] [14.9; -0.5; 0] [15.0; -0.5; 0] [15.1; -0.5; 0] [15.2; -0.5; 0] ...
%];

NO = 50;
X_O_ = [9; 1; 0];
for i = 1:NO
    alpha = pi + pi*i/NO;
    X_O(:,i) = X_O_ + 0.7*[cos(alpha); sin(alpha); 0];
end
NO1 = 30;
for i = 1:NO1
    X_O(:,NO+i) = X_O(:,NO) + [i*0.1; 0; 0];
end
NO = size(X_O, 2);
NO2 = 70;
for i = 1:NO2
    alpha = pi + pi*i/NO2;
    X_O(:,NO+i) = X_O(:,NO) + 2*[1+cos(alpha); sin(alpha); 0];
end

NO = size(X_O, 2);
for i = 1:NO
    X_O(:,i) = X_O(:,i) + 0.12*[randn(); randn(); 0];
end
%NO=0;
U_ = 2.;
W_ = 10.;
v0 = 0.8;
v_ = 1.;

P_ = [[1, 3]; [3, 9]];

t = t0:step:t1;
X0 = [x_0; y_0; z_0; thet_0; 0; v0];

P(:,1) = [0;0;0];
n = 10;
LX = 30;
StepX = LX / n;
for i=1:n-1
    P(:,i+1) = [i*StepX, 0, 0];
end
R = 3;
nr = 10;
alpha = -pi/2;
StepA = pi / nr;
for i=1:nr
    alpha = alpha + StepA;
    P(:,n+i) = P(:,n) + R*[cos(alpha); 1 + sin(alpha); 0];
%    disp(alpha);
%    disp(R*cos(alpha));
%    disp(R*sin(alpha));
end
for i=1:n - 1
    P(:,n+nr+i) = P(:, n+nr) - [StepX*i; 0; 0] ;
end
N = size(P,2);

MX = 1200;
MY = 200 + MX* 2*R / (LX+R);

global ip_;
ip_ = 1;

% Z = ode("adams", X0, t(1), t, wr);
% wr(0, X0)
% ret
f = @(t,X) (wr(t,X))';
[tsol, Z] = ode45(f,t, X0);
Z = Z'

NZ = size(Z, 2);
for i=1:NZ
%    [UU(i), UU1(i), XXsh(:,i), Tau_(:,i), Norm_(:,i)] = u(Z(:,i));
    [w_(i), a_(i), u_(i), V_(i), dV_(i), ddV_(i), Pot__(i), Grad_(:,i)] = U(Z(:,i));
end

figure();
plot(tsol, w_, 'g');
hl=legend(['w']);

figure();
plot(tsol, u_, 'b');
hl=legend(['u']);

%figure();
%plot(t, V_, 'b');
%hl=legend(['V']);
%figure();
%plot(t, dV_, 'b');
%hl=legend(['dV']);
%figure
%plot(t, ddV_, 'b');
%hl=legend(['ddV']);


figure();
plot(tsol, Pot__, 'r');
hl=legend(['Pot']);

figure();
plot(tsol, Grad_(1,:), 'b');
hl=legend(['Grad(1)']);
figure();
plot(tsol, Grad_(2,:), 'b');
hl=legend(['Grad(2)']);


figure();
plot(tsol, a_, 'r');
hl=legend(['a']);

figure();
plot(t, Z(6,:), 'r');
hl=legend(['v']);

figure;
hold on
% f.figure_size=[MX, MY];
% f.background = -2;
% e=gce();
% e.thickness = 2;
%comet([Z(1,:)' XXsh(1,:)' Z(5,:)'], [Z(2,:)' XXsh(2,:)' Z(6,:)'],"colors", [5 3 2]);
% comet([Z(1,:)' ], [Z(2,:)' ],"colors", [5 ]);
plot(Z(1,:), Z(2,:), 'ro')
ra = 0.1;
for io = 1:NO
%     xfarc(X_O(1,io)-ra/2., X_O(2,io)+ra/2.,2*ra,0.8*ra,0,360*64);
    plot(X_O(1,io)-ra/2., X_O(2,io)+ra/2., 'ko');
end
xmax = max(Z(1,:));
%e.thickness = 1;
plot([0, xmax], [0,0], 'b');

%xset("color",1);
%xset("thickness", 1);
%xpoly(P(1,:),P(2,:),"lines") 


function [Pot, Grad, Hess]=Potential(X)
    global U_
    global v_
    global b
    global bet
    global m_
    global P_
    global k_
    global k2
    global alp
    global mu
    global NO
    global NO1
    global NO2
    global v0
    global W_
    global X_O
    global rmin
    global aprev
    global delta0
    x = X(1);
    y = X(2);
    z = X(3);
    theta = X(4);
    u = X(5);
    v = X(6);
    ct = cos(theta);
    st = sin(theta);
    dX = v*[ct;st;0];

    XX = [x; y; z];

    Pot = 0;
    Grad = [0;0;0];
    Hess = [[0 0 0]; [0 0 0]; [0 0 0]];
    for io =1:NO
        X_Oc = X_O(:,io);
        dnorm = sqrt(dotp(XX - X_Oc, XX - X_Oc));
        dPot = dotp(XX - X_Oc, dX);
        if((dnorm < rmin) && (dPot < 0))
            Pot = Pot + 0.5*(rmin - dnorm)^2;
            Grad = Grad + (1. - rmin / dnorm) * (XX - X_Oc);
            Hess = Hess + eye(3) - (rmin / dnorm) * (eye(3) - (XX - X_Oc)*(XX - X_Oc)' * (1. / dnorm^2));
%             Pot = Pot + Vmin - log(dnorm - delta0^2);
%             Grad = Grad - (2. / (dnorm - delta0^2)) * (XX - X_Oc);
%             Hess = Hess + (2. / (dnorm - delta0^2)) * ((2. / (dnorm - delta0^2)) * (XX - X_Oc)*(XX - X_Oc)' - eye(3) );
        end
    end
%     Pot = log(rmin*rmin) - log(dnorm);
%     Grad = -(XX - X_O) * 2. / dnorm;
end

function [w, a, u, V, dV, ddV, Pot_, Grad]=U(X)
    global U_
    global v_
    global b
    global bet
    global m_
    global P_
    global k_
    global k2
    global alp
    global mu
    global W_


    x = X(1);
    y = X(2);
    z = X(3);
    theta = X(4);
    u = X(5);
    u = sat(U_, u);
    v = X(6);
    ct = cos(theta);
    st = sin(theta);

    [Pot_, Grad, Hess] = Potential(X);
    
    dpo = Grad'*[ct; st; 0];
% %    if(dpo < 0)
% %        dpo = 0;
% %    end
    a = (b*(v_ - v) - bet*dpo) / m_;
    da = 0; % -0.5 * a;

    dy = v*st;
    ddy = v*v*ct*u + m_*st*a;
    V = 0.5*[y, theta] * P_ * [y; theta];
    dV = [y, theta] * P_ * [dy; v*u];
    dX = v*[ct;st;0];
    dPot = dotp(Grad, dX);
    A_ = dX'*Hess*dX;
    B_ = a*m_*Grad'*[ct;st;0];
    E_ = k_*v*Grad'*[-st;ct;0];
    
    A = [dy, v*u] * P_ * [dy; v*u];
    B = [y, theta] * P_ * [ddy; -v*k2*u];
    E = [y, theta] * P_ * [0; v*k_] + alp*E_;

    w = -(mu*mu*(V + alp*Pot_) + 2*mu*(dV + alp*dPot) + A + B + alp*A_ + alp*B_)*E / (1.e-6 + E*E);
    if(abs(ct) < 0.1)
        w = -W_*sign(dy);
    end
    
    w = sat(W_, w);
    ddV = A + w*B;
% %    disp("w", w)
end

function s = sat(am, a)
    s = a;
    if(a > am)
        s = am;
    end;
    if(a < -am)
        s = -am;
    end;
end

function dy=wr(t, X)
    global U_
    global v_
    global b
    global bet
    global m_
    global P_
    global k_
    global k2
    global alp
    global mu
    global W_
    X
    x = X(1);
    y = X(2);
    z = X(3);
    theta = X(4);
    u = X(5);
    u = sat(U_, u);

    v = X(6);
    ct = cos(theta);
    st = sin(theta);
    [w, a]=U(X);
    dy(1) = v*ct;
    dy(2) = v*st;
    dy(3) = 0;
    dy(4) = v*u;
    dy(5) = k_*w;
    dy(6) = m_*a;
% %    disp("dy6",dy(6))
end

function s=dotp(X, Y)
    s = X(1)*Y(1) + X(2)*Y(2) + X(3)*Y(3);
end

function j=incr(i)
    j = i+1;
    if(j > N)
        j = j - N;
    end
end

function [in, Y, dmin] = near1(ip, X)
global ip_;

for i = ip:ip+N-2
    j = i;
    if(j > N)
        j = j - N;
    end
    A = P(:,j);
    B = P(:,incr(j));
    L = B - A;
% %    disp(A)
% %    disp(B)
    tau = -dotp(A-X, L) / dotp(L, L);
    if((tau >= 0) && (tau <= 1))
        in = j; 
        Y = A + tau*L;
        dmin = sqrt(dotp(Y-X, Y-X));
        return;
    end
end
in = 1;
Y = X;
dmin = -1;
end

function [in, Y, dmin] = near(ip, X)
in = 0;
dmin = 1.e12;
global ip_;
%disp("ip", ip)
done = 0;
in = ip;
Y = X;
for i = ip:ip + N-2
    j = i;
    if(j > N)
        j = j - N;
    end
    A = P(:,j);
    B = P(:,incr(j));
    L = B - A;
    tau = -dotp(A-X, L) / dotp(L, L);
    if(tau < 0)
        tau = 0;
    end
    if(tau > 1)
        tau = 1;
    end
    XC = A + tau*L;
    dcur = dotp(XC-X, XC-X);
    if(dcur <= dmin)
        in = j;
        Y = XC;
        dmin = dcur;
        done = 1;
    end
%    disp("--", ip, X', in, Y');
end
dmin = sqrt(dmin)
end


