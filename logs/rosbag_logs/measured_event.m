function [is_event, stamp, measure] = measured_event(t, t_lst, stamps, measures)
% MEASURED_EVENT is for play logs.
% It returns is_event = 0 and empty measures vector if there no measures
% since last call in stamped_measures.

is_event = 0;
stamp = nan;
measure = [];

n_stamps = length(stamps);
n_mes = length(measures(:,1));
if n_stamps ~= n_mes
    error('measured_event:incorrectInput', 'Error. length(stamps) ~= length(measures)')
end

if t < stamps(1)
    return;
end
if t > stamps(end)
    return;
end

multy_event_check = -1;
for i = 1:length(stamps)
    if t_lst < stamps(i) &  t > stamps(i)
        is_event = 1;
        stamp = stamps(i);
        measure = measures(i, :);
        multy_event_check = multy_event_check + 1;
    end
end

if multy_event_check > 0
    warning('measured_event: multyply events detected, try set a smoller t step')
end

end

