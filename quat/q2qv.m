function qv = q2qv(q)

qv = q(2:4);
if q(1) < 0
    qv_n = norm(qv);
    sin_half_alpha = qv_n;
    pin = qv / qv_n;
    alpha = 2 * asin(sin_half_alpha);
    alpha_new = -2*pi + alpha;
    qv = -pin * sin(alpha_new / 2);
end

end

