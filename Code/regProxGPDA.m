function [x, s] = regProxGPDA(t, q_in, x_in)
%PROXGPDA Summary of this function goes here
%   Detailed explanation goes here
    global T
    global D
    global beta
    global N
    global n
    global epsilon
    global D
    global Lplus
    global A
    global Delay
    global x0
    global mu
    global ai
    global method
    global s_i
    nu = 1;
    ell = 1;
    
    x_i = zeros(n,N);
    df = costGradFunction(x_in);
    while norm(df,2) <= epsilon
        for ii = 1:N
            for jj = 1:N
                if A(ii,jj) == 1 || A(ii,jj) == -1
                    si = 1/(2*beta*D(ii,ii))*(beta*x_in(:,jj) - costGradFunction(x_in(:,ii), x0(:,ii), ai(ii), method) - beta*mu);
                    x_o = si + q_in(:,ii);
                    
                    if ell < Delay
                        ell = ell + 1;
                    else
                        mu(ii) = mu(ii) + beta*(x_in(:,ii) - x_in(:,jj));
                        ell = 0;
                    end
                end
            end
            s_i(:,ii) = si;
            x_i(:,ii) = x_o;
            nu = nu + 1;
        end
    end
    x = x_i;
    s = s_i;
end

