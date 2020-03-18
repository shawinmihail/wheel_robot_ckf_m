clc
clear


q = [100;10;0;0]
for i = 1:1000
    q(4) = i/10;
    qn = q/norm(q);
    e = quat2Eul(qn)*180/pi;
    a(i) = e(3);
end
plot(a)

