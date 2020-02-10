function [q] = qv2q(qv)
% suppose q0 > 0;
sq_q0 = 1 - qv(1)^2 - qv(2)^2 - qv(3)^2;

if sq_q0 < 0
    sq_q0 = 0;
end

q0 = sqrt(sq_q0);
q = [q0; qv];
q = q / norm(q);
end
