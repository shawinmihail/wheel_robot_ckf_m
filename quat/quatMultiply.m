function p = quatMultiply( q, r )

qw = r(1) * q(1) - r(2) * q(2) - r(3) * q(3) - r(4) * q(4);
q1 = r(1) * q(2) + r(2) * q(1) - r(3) * q(4) + r(4) * q(3);
q2 = r(1) * q(3) + r(2) * q(4) + r(3) * q(1) - r(4) * q(2);
q3 = r(1) * q(4) - r(2) * q(3) + r(3) * q(2) + r(4) * q(1);
p = [qw;q1;q2;q3];

end

