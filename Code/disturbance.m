function di = disturbance(t)
%DISTURBANCE Summary of this function goes here
%   Detailed explanation goes here
global N
    for i = 1:N
        di(:,i) = [0.1*cos(t + i*pi/3); 0.1*sin(t + i*pi/3); 0];
    end
end

