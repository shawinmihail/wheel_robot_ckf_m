function [ qWrapped] = quatWrapPi( q )

eps = 1e-6;
if q(1) < 0
    alpha = 2*acos(q(1));
    q(1) = abs(q);
    pin = [q(2);q(3);q(4)];
    npin = norm(pin);
    if npin < eps
        qWrapped = [1;0;0;0];
        return;
    end
    pin = pin/npin;
    


end

