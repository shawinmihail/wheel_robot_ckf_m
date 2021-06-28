function res = integrator(val, dt, lim, last_res)
res = last_res + val * dt;
res = min(max(res, -lim), lim);
end

