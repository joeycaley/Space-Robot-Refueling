
%% Initialization
clear all;
close all;

addpath('mr')

deg2rad = pi/180;
R_earth = 6378e03;
minGap = 10*deg2rad;
mu_Earth = 3.986004*10^14;

%% Orbital Rendezvous

% choose solutions to explore
k = 1:20;

% decide if you want to do animations
animationCheck = 1;

% define orbit
a = 400e03 + R_earth;
e = 0;
i = 30*deg2rad; % Example test
RAAN = 120*deg2rad; % Example test
% i = 2*pi*rand; % Generalized randomized orbit
% RAAN = 2*pi*rand; % Generalized randomized orbit
argP = 0;

fg = 240*deg2rad; % Example test
% fg = 2*pi*rand; % Generalized randomized orbital position

fprintf("Goal orbit: i = %f degrees, at RAAN = %f degrees\n",i/deg2rad,RAAN/deg2rad)
fprintf("Goal Spacecraft will be initialized at f0 = %f degrees\n\n", fg/deg2rad)

%-INCLINATION & RAAN CHANGE------------------------------------------------
f = 0*deg2rad; % Example test
% f = 2*pi*rand; % Generalized randomized orbit starting position

% calculate deltaV for plane change, time to get there, and distance traveled
[deltaVTotal, tElapsedTotal, thetaTrav] = planeChange(a,i,RAAN,f);

% calculate positoin of orbit plane change
thetaTrav = 0:.05:thetaTrav;

xTrav = a*cos(thetaTrav);
yTrav = a*sin(thetaTrav);
zTrav = zeros(1,length(thetaTrav));

% vmag = sqrt(mu_Earth/a);
vmag = 3e06;

vz = vmag*sin(i);
vxy = vmag*cos(i);
vy = vxy*cos(RAAN);
vx = -vxy*sin(RAAN);

%-PHASING MANUEVER---------------------------------------------------------

% g refers to goal, the spacecraft we are trying to reach
% c refers to chaser, the spacecraft we control
fc = 0;

% create orbital elements
OE_g = [a e i RAAN argP fg];
OE_c = [a e i RAAN argP fc];

% run all solutions
for i = 1:length(k)
    [dirOfApr, deltaV(i), tElapsed(i), possCheck(i), OE_t(i,:)] = interceptTraj(OE_g,OE_c,k(i));
end

% remove impossible solutions
k = k(possCheck);
deltaV = deltaV(possCheck);
tElapsed = tElapsed(possCheck);
OE_t = OE_t(possCheck,:);

% Create sum vectors
deltaVTotal = deltaVTotal + 2*deltaV;
tElapsedTotal = tElapsedTotal + tElapsed;

% if you wanna see animations
if animationCheck == 1
    % Create position values for quickest phasing manuever
    at = OE_t(1,1);
    et = OE_t(1,2);
    Pt = 2*pi*sqrt(at^3/mu_Earth);
    
    t_t = 0:60:tElapsed(1);
    ft = zeros(1,length(t_t));
    n = sqrt(mu_Earth/at^3);
    
    % check which for outer vs inner orbit to see whether starting at
    % peri/apoapsis
    if dirOfApr == 0
        ft(1) = pi;
    else
        ft(1) = 0;
    end

    % calculate all true anamolies of transfer orbit
    for i = 2:length(t_t)
        E0 = 2*atan(sqrt((1-et)/(1+et)) * tan(ft(i-1)/2));
        M0 = E0 - et*sin(E0);
        M = M0 + n*(t_t(i) - t_t(i-1));
        E = newtonRaphsonKepEllip(M,et);
        ft(i) = 2*atan(sqrt((1+et)/(1-et)) * tan(E/2));
    end

    rt = (at*(1-et^2))./(1+et*cos(ft));
    
    xt = rt.*cos(ft);
    yt = rt.*sin(ft);

    % calculate true anamolies of goal spacecraft's orbit
    fog = linspace(fg, 2*pi + (k(1)-1)*2*pi, length(ft));

    xog = a*cos(fog);
    yog = a*sin(fog);

end

% output direction of approach
if dirOfApr == 0
    fprintf("Chaser spacecraft approaches from bottom with inner transfer orbit\n")
    T = 0; %passed to near-field planning
else
    fprintf("Chaser spacecraft approaches from top with outer transfer orbit\n")
    T = 1; %passed to near-field planning
end

%-PLOTTING-----------------------------------------------------------------

% plot everything
plotRendezvous


% pause before continuing
input("Press any key to continue\n")

% close all to avoid creating issues with future sections
close all

%% Local Path Planning to Satellite
%This portion of the code takes the location of the spcaecraft from the
%orbital mechanics, and calculates a trajectory for the spacecraft to get
%within 25 m of the target craft


%From Orbital mechanics, recieve top / bottom orientation of craft
if T == 0
    start.x = randi([1,10]);
    start.y = randi([1,10]);
    start.z = randi([250,485]);
end

if T == 1
    start.x = randi([1,10]);
    start.y = randi([1,10]);
    start.z = randi([1,250]);
end


%Goal point randomly generated for simulation, in actual application, goal
%location is known.
goal.x = randi([350,490]);
goal.y = randi([350,490]);
goal.z = randi([350,490]);

[EndLoc, cost, iter] = RRTstar3D(start,goal);

%The EndLoc contains the x, y, and z coordinates of the craft after
%following the RRT* trajectory. At this location, the craft will achieve a
%velocity of 0 relative to the target spacecraft, and will have no shift in
%roation, only location

%Now, the craft body will be set as the base frame, and the target craft
%coordinates will be transformed into the base frame
global x_end;
x_end = [EndLoc.x-goal.x;EndLoc.y-goal.y; EndLoc.z-goal.z];

%% Robotic Arm Manuevering

%%
model = load_model();
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


