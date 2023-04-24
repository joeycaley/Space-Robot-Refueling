function E_final = newtonRaphsonKepEllip(M, e)
% Find eccentric anamoly, E (rad), for a given mean anamoly, M (rad) and
% eccentricity, e for an elliptical orbit.

% POSSIBLY VARARGIN TO DEFINE EPSILON AND MAX # OF ITERATIONS, AND MAYBE
% FUNCTION TO SOLVE?

epsilon = 10^-10; %stopping criteria
N = 10; %max number of iterations

E(1) = M;
k = 1;

Eflag = 0;

while Eflag == 0
    deltaE(k+1) = (M - (E(k) - e*sin(E(k))))/(1-e*cos(E(k)));
    E(k+1) = E(k) + deltaE(k+1);
    
    if abs(deltaE(k+1)) < epsilon || k > N
        Eflag = 1;
    else
        k = k + 1;
    end
end

if k > N
    fprintf("exceeded max number of iterations\n")
end

E_final = E(end);

end