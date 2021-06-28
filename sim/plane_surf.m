function [surf, grad_surf] = plane_surf()
surf = @(x,y)(x.*0 + y.*0);
grad_surf = @(r) [0; 0; 1];
end