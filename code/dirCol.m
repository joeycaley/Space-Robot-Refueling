clear;clc

N = 20;
z0 = rand((N+1)*(2*model.nDof+model.nInputs) + 1,1);
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point',...
    'MaxIterations',5000,'MaxFunctionEvaluations',3000000);

z = fmincon(@(z)cost(z,N),z0,[],[],[],[],[],[],@(z)nonlcon(z,N),options);


for i=0:N
    x(:,i+1) = z(4*i+1:4*i+4)';
    u(i+1)   = z(4*(N+1) + (i+1));
    T        = z(end);
end

q = x(1:2,:);
dq = x(3:4,:);
t = 0:T/N:T;

%%
figure(10);clf;

subplot(2,2,1);
plot(t,x(1,:));
title('Cart Positioin');
grid on;

subplot(2,2,2);
plot(t,x(2,:));
title('Pendulum Angle');
grid on;

subplot(2,2,3);
plot(t,x(3,:));
title('Cart Velocity');
grid on;

subplot(2,2,4);
plot(t,x(4,:));
title('Pendulum Velocity');
grid on;

%% animation
animation(q,T/N);