%% Initialization
addpath('mr')
model = load_model();

%%
N = 10;
z0 = rand((N+1)*(2*model.nDof+model.nInputs) + 1,1);
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point',...
    'MaxIterations',1000,'MaxFunctionEvaluations',3000000);
x_start = rand(3,1);
z = fmincon(@(z)cost(z,N,model),z0,[],[],[],[],[],[],@(z)nonlcon(z,N,model),options);

%%
[x,u,T] = extract(z,N,model);
t = 0:T/N:T;

%% Plots
figure();

subplot(2,3,1);
plot(t,x(1,:));
title('Angle of Joint 1');
xlabel('Time [s]')
ylabel('Angle \theta_1 [rad]')
grid on;

subplot(2,3,2);
plot(t,x(2,:));
title('Angle of Joint 2');
xlabel('Time [s]')
ylabel('Angle \theta_2 [rad]')
grid on;

subplot(2,3,3);
plot(t,x(3,:));
title('Angle of Joint 3');
xlabel('Time [s]')
ylabel('Angle \theta_3 [rad]')
grid on;

subplot(2,3,4);
plot(t,x(4,:));
title('Angle of Joint 4');
xlabel('Time [s]')
ylabel('Angle \theta_4 [rad]')
grid on;

subplot(2,3,5);
plot(t,x(4,:));
title('Angle of Joint 5');
xlabel('Time [s]')
ylabel('Angle \theta_5 [rad]')
grid on;

subplot(2,3,6);
plot(t,x(4,:));
title('Angle of Joint 6');
xlabel('Time [s]')
ylabel('Angle \theta_6 [rad]')
grid on;

%% Plots
figure();

subplot(2,3,1);
plot(t,x(7,:));
title('Velocity of Joint 1');
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
grid on;

subplot(2,3,2);
plot(t,x(8,:));
title('Velocity of Joint 2');
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
grid on;

subplot(2,3,3);
plot(t,x(9,:));
title('Velocity of Joint 3');
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
grid on;

subplot(2,3,4);
plot(t,x(10,:));
title('Velocity of Joint 4');
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
grid on;

subplot(2,3,5);
plot(t,x(11,:));
title('Velocity of Joint 5');
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
grid on;

subplot(2,3,6);
plot(t,x(12,:));
title('Velocity of Joint 6');
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
grid on;
