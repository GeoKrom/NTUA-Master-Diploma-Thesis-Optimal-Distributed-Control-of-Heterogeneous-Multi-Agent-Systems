function [value,isterminal,direction] = event(t,x)
    global T k
    value =   t - k*T;         % Fire every T
    isterminal = 1;            % Stop integration
    direction = -1;             % Positive zero-crossing
end