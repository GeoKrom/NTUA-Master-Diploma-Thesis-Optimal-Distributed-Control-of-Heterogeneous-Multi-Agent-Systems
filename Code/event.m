function [value,isterminal,direction] = event(t, x)
    global T k n N triggerTime
    triggerTime = k*T;
    value = t - triggerTime;
    isterminal = 1;
    direction = 0;
end