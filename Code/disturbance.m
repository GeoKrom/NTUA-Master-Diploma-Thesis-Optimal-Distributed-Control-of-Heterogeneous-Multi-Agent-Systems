function d = disturbance(t)
%DISTURBANCE Summary of this function goes here
%   Detailed explanation goes here
    global N
    global n
    global d
    di = zeros(n,N);
    for i = 1:N
        di(:,i) = [0.1*cos(i*t*pi/3); 0.1*sin(i*t*pi/3); 0];
    end
    % disp(di);
    d = reshape(di,[n*N,1]);
end

