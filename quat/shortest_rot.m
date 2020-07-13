function [sa] = shortest_rot(a)

for i = 1:length(a)
    sa(i) = a(i);
    if sa(i) > 180
        sa(i) = sa(i) - 360;
    end
end

end

