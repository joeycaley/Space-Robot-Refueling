function f = cost(z,N,model)

% N = 40;

[x,u,T] = extract(z,N,model);

f = 0;
h = T/N;
for i=1:N
    f = f+0.5*h*(u(:,i)'*u(:,i) + u(:,i+1)'*u(:,i+1));
end

% f = f + 10*T;
end