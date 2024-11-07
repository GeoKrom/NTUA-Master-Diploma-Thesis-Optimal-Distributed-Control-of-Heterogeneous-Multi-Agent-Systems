function r = rho(t)
% RHO function converts a discrete signal to a continuous signal
%   The method that is used is called Zero-Order Hold that converts a
%   discrete signal with a sampled period to a continuous.
    global T
    global d_max

    if t < d_max
        r = 0;
    elseif t >= d_max && t <= T
        r = (t - d_max)/(T - d_max) - 1/(2*pi)*sin(2*pi*(t - d_max)/(T - d_max));
    else
        r = 1;
    end
end

