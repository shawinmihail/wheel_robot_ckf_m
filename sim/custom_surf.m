function [surf, grad_surf] = custom_surf()
surf = @(x,y)(x.*x/30 -1*y.*y/30 + 1);
grad_surf = @(r) [-r(1)/15; +r(2)/15; 1];
end