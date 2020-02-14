function [surf, grad_surf] = custom_surf()
surf = @(x,y)(x.*x/300 -1*y.*y/300 + 1);
grad_surf = @(r) [-r(1)/150; +r(2)/150; 1];

% surf = @(x,y)(x.*0.2 + y.*0.5 + 1);
% grad_surf = @(r) [-0.2; -0.5; 1];
end