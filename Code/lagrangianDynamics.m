function dv = lagrangianDynamics(t, state, u)
    global N
    global n
    global M
    global C
    global G
    global v_dot
    v = state((n*N+1):2*n*N);
   
    
    for i=1:n*N
        v_dot(i) = inv(kron(M,ones(n)))*(u + disturbance(t) - C*v(i) - G);
    end
    dv = v_dot;
end

