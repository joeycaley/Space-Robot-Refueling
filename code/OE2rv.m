function [r_I, v_I] = OE2rv(OE)
% Convert orbital data from orbital elements [semimajor axis (m),
% eccentricity (unitless), inclination (radians), argument of periapsis
% (radians), right ascension of ascending node (radians), true anamoly
% (radians)] to radius (m) and velocity (m/s) vectors in the Earth centered
% inertial frame

a =    OE(1);
e =    OE(2);
i =    OE(3);
RAAN = OE(4);
argP = OE(5);
f =    OE(6);

mu = 3.986004*10^14;

% parameter
p = a*(1-e^2); %meters

% magnitude of angular momentum vector
h_mag = sqrt(mu*p); %m^2/s

% energy
energy = -mu/(2*a); %m^2/s^2

% magnitude of r and v
r_mag = p/(1 + e*cos(f)); %meters     POTENTIAL TO IMPROVEEEEEEEEEEEEEEEEEEE
v_mag = sqrt(2*(energy + mu/r_mag));

% determination of r and v in orbital frame
r_O = [r_mag*cos(f); r_mag*sin(f); 0];

f_dot = h_mag/r_mag^2;
r_dot = h_mag*e/p*sin(f);

v_O = [...
    r_dot*cos(f) - r_mag*f_dot*sin(f); ...
    r_dot*sin(f) + r_mag*f_dot*cos(f); ...
    0];

% create the DCM using 3-1-3 euler angles 
cw = cos(argP);
co = cos(RAAN);
ci = cos(i);
sw = sin(argP);
so = sin(RAAN);
si = sin(i);

R_OI = [cw*co-sw*ci*so   , cw*so+sw*ci*co , sw*si; ...
        -(sw*co+cw*ci*so), -sw*so+cw*ci*co, cw*si; ...
        si*so            , -si*co         , ci];
R_IO = R_OI';

% transform r and v from orbital to inertial frame
r_I = R_IO*r_O;
v_I = R_IO*v_O;

end