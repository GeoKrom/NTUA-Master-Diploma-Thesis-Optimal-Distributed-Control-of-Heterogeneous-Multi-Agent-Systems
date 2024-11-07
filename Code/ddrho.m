function ddr = ddrho(t)
%DDRHO Summary of this function goes here
%   Detailed explanation goes here
    global T
    global d_max

    if t < d_max
        ddr = 0;
    elseif t >= d_max && t <= T
        ddr = (2*pi)/(T - d_max)^2*sin(2*pi*(t - d_max)/(T - d_max));
    else
        ddr = 0;
    end
end

