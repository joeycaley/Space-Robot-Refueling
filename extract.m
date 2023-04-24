function [x,u,T] = extract(z,N,model)
% This function will extract the state, control, and time variables from
% the optimization variable vector (z)

nDof = model.nDof;
nInputs = model.nInputs;


x = reshape(z(1:2*nDof*(N+1)),2*nDof,(N+1));
u = reshape(z(2*nDof*(N+1)+(1:nInputs*(N+1))),nInputs,(N+1));
T = z(end);


end

