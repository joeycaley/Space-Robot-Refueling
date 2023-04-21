clear all;
close all;

%% Orbital Rendevouz

%% Local Path Planning to Satellite

%% Robotic Arm Manuevering
%% Initialization
addpath('mr')
model = load_model();

%%
N = 20;
z0 = rand((N+1)*(2*model.nDof+model.nInputs) + 1,1);
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point',...
    'MaxIterations',750,'MaxFunctionEvaluations',3000000);
x_start = rand(3,1);
z = fmincon(@(z)cost(z,N,model),z0,[],[],[],[],[],[],@(z)nonlcon(z,N,model),options);

%%
[x,u,T] = extract(z,N,model);
