function dr = drho(t)
%DRHO Summary of this function goes here
%   Detailed explanation goes here
    global T
    global d_max k

    if t <= d_max
        dr = 0;
    elseif t > d_max && t <= T
        dr = (1/(T - d_max))*(1 - cos(2*pi*(t - d_max)/(T - d_max)));
    end
end

