function fi = costGradFunction(x, x_init, a_i, type)
%COSTFUNCTION returns a Nonconvex function
    global tau

    if strcmp(type,'simple')
        fi = exp(-a_i*x)/(1 + exp(-a_i*x))^2;
    
    else if strcmp(type, 'obstacle avoidance')
        fi = norm(x - x_init) - tau/(tau + 1).*norm(x - x_obs);
    
    end
end

