function [surf, grad_surf] = custom_surf()
surf = @(x,y)(x.*x/900 -1*y.*y/800 + 1);
grad_surf = @(r) [-r(1)/450; +r(2)/400; 1];

% surf = @(x,y)(x.*x/100 -1*y.*y/100 + 1);
% grad_surf = @(r) [-r(1)/50; +r(2)/50; 1];

% surf = @(x,y)(x.*0.2 + y.*0.5 + 1);
% grad_surf = @(r) [-0.2; -0.5; 1];
end