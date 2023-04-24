function [deltaV, tElapsed,theta] = planeChange(a,i,RAAN,f)

% constants
mu_Earth = 3.986004*10^14;

% v calculation
v = sqrt(mu_Earth/a);

% inclination change requirement
deltaV = 2*v*sin(i/2);

% difference
theta = RAAN - f;

if theta < 1
    theta = theta + 2*pi;
end

% time to get there
tElapsed = theta*sqrt(a^3/mu_Earth);

end