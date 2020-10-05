function val = Vfun(z1, z2, theta, umax, c, P)
val = [];

for i = 1:size(z2)
    z = [z1(i);z2(i)];
    sigma = c'*z;
    if sigma >= umax*(1+z2(i)*z2(i))^1.5   
        addval = theta*(umax*(1+z2(i)*z2(i))^1.5)*(sigma-0.5*umax*(1+z2(i)*z2(i))^1.5);
    elseif sigma <= -umax*(1+z2(i)*z2(i))^1.5   
        addval = theta*(umax*(1+z2(i)*z2(i))^1.5)*(-sigma-0.5*umax*(1+z2(i)*z2(i))^1.5);
    else %if abs(sigma) < umax*(1+z2*z2)^1.5
        addval = theta*(sigma^2)/2;
    end

    val = [val; z'*P*z + addval];
end
end