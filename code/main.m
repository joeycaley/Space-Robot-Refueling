clear all;
close all;

deg2rad = pi/180;
R_earth = 6378e03;
minGap = 10*deg2rad;
mu_Earth = 3.986004*10^14;

%% Orbital Rendezvous

% choose solutions to explore
k = 1:20;

% define orbit
a = 400e03 + R_earth;
e = 0;
i = 30*deg2rad;
% i = 2*pi*rand;
% RAAN = 2*pi*rand;
RAAN = 270*deg2rad;
argP = 0;

%-INCLINATION & RAAN CHANGE------------------------------------------------
% f = 2*pi*rand;
f = 60*deg2rad;

[deltaVTotal, tElapsedTotal,thetaTrav] = planeChange(a,i,RAAN,f);

thetaTrav = 0:.05:thetaTrav;

xTrav = a*cos(thetaTrav);
yTrav = a*sin(thetaTrav);
zTrav = zeros(1,length(thetaTrav));

%-PHASING MANUEVER---------------------------------------------------------

% g refers to goal, the spacecraft we are trying to reach
% c refers to chaser, the spacecraft we control
% fg = 2*pi*rand;
fg = 170*deg2rad;
fc = 0;

% create orbital elements
OE_g = [a e i RAAN argP fg];
OE_c = [a e i RAAN argP fc];

% run all solutions
for i = 1:length(k)
    [dirOfApr, deltaV(i), tElapsed(i), possCheck(i)] = interceptTraj(OE_g,OE_c,k(i));
end

% remove impossible solutions
k = k(possCheck);
deltaV = deltaV(possCheck);
tElapsed = tElapsed(possCheck);

% Create sum vectors
deltaVTotal = deltaVTotal + 2*deltaV;
tElapsedTotal = tElapsedTotal + tElapsed;

% output direction of approach
if dirOfApr == 0
    fprintf("Chaser spacecraft approaches from bottom with inner transfer orbit\n")
else
    fprintf("Chaser spacecraft approaches from top with outer transfer orbit\n")
end

% plot time to intercept
figure
plot(k,tElapsedTotal./3600,'LineWidth',1.5)
xlabel("k")
ylabel("Time (hr)")
title("Total Time to Reach Serviced Spacecraft")
grid on

% plot delta V required
figure
plot(k,deltaVTotal,'LineWidth',1.5)
xlabel("k")
ylabel("\Deltav Required (m/s)")
title("Total \Deltav to Reach Serviced Spacecraft")
grid on

% figure
% for i = 1:length(thetaTrav)
%     scatter3(0,0,0,'SizeData',20)
%     plot3(xTrav(1:i),yTrav(1:i),zTrav(1:i))
%     grid on
%     pause(.05)
% end

%% Local Path Planning to Satellite

%% Robotic Arm Manuevering
%% Initialization
% addpath('mr')
% model = load_model();
% 
% %%
% N = 20;
% z0 = rand((N+1)*(2*model.nDof+model.nInputs) + 1,1);
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point',...
%     'MaxIterations',750,'MaxFunctionEvaluations',3000000);
% x_start = rand(3,1);
% z = fmincon(@(z)cost(z,N,model),z0,[],[],[],[],[],[],@(z)nonlcon(z,N,model),options);
% 
% %%
% [x,u,T] = extract(z,N,model);
