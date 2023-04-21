function dx = dynamicsEqn(t,x,u,model)

Mlist = model.Mlist;
Glist = model.Glist;
Slist = model.Slist;
nDof  = model.nDof;

% extract joint angles and velocities from the states
q = x(1:nDof);
dq = x(nDof+1:2*nDof);
% mass matrix
M = MassMatrix(q,Mlist,Glist,Slist);
% corilios force vector = C(q,dq)*dq
C = VelQuadraticForces(q,dq,Mlist,Glist,Slist);
% gravity force vector
g = [0,0,-9.81];
G = GravityForces(q,g,Mlist,Glist,Slist);

% Kp = diag([1000,1000,1000,100,100,100]);
% Kd = diag([40,40,40,40,40,40]);
% 
% qd = [pi/2;0;0;0;0;0];
% dqd = [0;0;0;0;0;0];

% torque = Kp*(qd-q) + Kd*(dqd-dq);%[2;2;2;2;2;2]; % open-loop constant torques
% torque = 2*ones(6,1);

ddq = M\(u-C-G);

dx = [dq;ddq];


end