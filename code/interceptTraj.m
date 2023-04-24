function [dirOfApr, deltaV, tElapsed, possCheck] = interceptTraj(OE_g, OE_c, k)
% constants
mu_Earth = 3.986004*10^14;

% extract values from orbital elements
a = OE_g(1);
R = a;

fg = OE_g(end);
fc = OE_c(end);
    
Pog = 2*pi*sqrt(a^3/mu_Earth);
nog = 2*pi/Pog;

% calculate beta
beta = fg - fc;

% assume inner orbit with an approach from the bottom
dirOfApr = 0;

% check if goal is ahead of chaser
if beta > 0
    
    % check if goal spacecraft is ahead of chase spacecraft by more than a
    % half period
    if beta >= pi
        % need to slow down
        % outer orbit, approach from top
        dirOfApr = 1;
        
        [deltaV, tElapsed, at] = outerOrbit(k,beta,Pog,nog,R,mu_Earth);
    else
        % need to speed up
        % inner orbit, approach from bottom
        dirOfApr = 0;

        [deltaV, tElapsed, at] = innerOrbit(k,beta,Pog,nog,R,mu_Earth);
    end

% beta is negative, chaser is ahead of goal
else
    % eliminate negative sign
    beta = abs(beta);

    % check if chaser is ahead of goal by more than a half period
    if beta > pi
        % need to speed up
        % inner orbit, approach from bottom
        dirOfApr = 0;

        [deltaV, tElapsed, at] = innerOrbit(k,beta,Pog,nog,R,mu_Earth);
    else
        % need to slow down
        % outer orbit, approach from top
        dirOfApr = 1;

        [deltaV, tElapsed, at] = outerOrbit(k,beta,Pog,nog,R,mu_Earth);
    end
end

possCheck = isItPossible(deltaV,a,at);

end


function [deltaV, tElapsed, at] = outerOrbit(k,beta,Pog,nog,R,mu_Earth)

Pt = Pog + (2*pi - beta)/(k*nog);
at = (mu_Earth*Pt^2/(4*pi^2))^(1/3);

deltaV = sqrt(2*mu_Earth/R - mu_Earth/at) - sqrt(mu_Earth/R);
tElapsed = k*Pt;

end

function [deltaV, tElapsed, at] = innerOrbit(k,beta,Pog,nog,R,mu_Earth)

Pt = Pog - beta/(k*nog);
at = (mu_Earth*Pt^2/(4*pi^2))^(1/3);

deltaV = sqrt(mu_Earth/R) - sqrt(2*mu_Earth/R - mu_Earth/at);
tElapsed = k*Pt;

end

function possCheck = isItPossible(deltaV, a,at)

possCheck = deltaV == real(deltaV);

if possCheck == true
    delta = 100e03;
    R_earth = 6378e03;
    
    possCheck = 2*at - a > R_earth + delta;
end

end