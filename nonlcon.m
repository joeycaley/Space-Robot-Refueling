function [c,ceq] = nonlcon(z,N,x_end,model)

umax = [150; 150; 150; 28; 28; 28];
dqmax = [3.15; 3.15; 3.15; 3.2; 3.2; 3.2];

dx = zeros(2*model.nDof,N+1);

[x,u,T] = extract(z,N,model);

for i=1:N+1
    dx(:,i) = dynamicsEqn([],x(:,i),u(:,i),model);
end




h = T/N;
c = zeros((2*model.nInputs+2*model.nDof)*(N+1),1);
for i=0:N
    %     q = x(1:model.nDof,i+1);
    % forward kinematics to compute end effector position P
    %     P = FKinSpace(q)
    c((2*model.nInputs+2*model.nDof)*i+1:(2*model.nInputs+2*model.nDof)*(i+1)) = [
        x(model.nDof+1:2*model.nDof,i+1) - dqmax;
        -x(model.nDof+1:2*model.nDof,i+1) - dqmax;
        u(:,i+1)-umax;
        -u(:,i+1)-umax];
    % add obstacle constraints here
    %         R_obs - sqrt(P-P_obs)];
end


ceq = zeros(2*model.nDof*(N-1),1);

for i=0:N-1
    ceq(2*model.nDof*i+1:2*model.nDof*(i+1)) = x(:,i+2)-x(:,i+1)-0.5*h*(dx(:,i+2)+dx(:,i+1));    
end


% T1d = [   -1.0000   -0.0000    0.0000    0.8173
%     0    0.0000    1.0000    0.1915
%     -0.0000    1.0000   -0.0000   -0.0055
%     0         0         0    1.0000];
% 
% T2d = [   -0.0148    0.9915   -0.1294   -0.0528
%     0.9744   -0.0148   -0.2241    0.1268
%     -0.2241   -0.1294   -0.9659   -0.7305
%     0         0         0    1.0000];
% 
% T1 = FKinSpace(model.M, model.Slist, x(1:model.nDof,1));
% T2 = FKinSpace(model.M, model.Slist, x(1:model.nDof,N+1));
x_start = [0;0;0]; % end effector position
T_end_start = FKinSpace(model.M,model.Slist,x(1:model.nDof,1)); %node 1 
p_end_start = T_end_start(1:3,4);

%x_end = [13.55;-10.2692;-3.9041]; % end effector position

T_end_end = FKinSpace(model.M,model.Slist,x(1:model.nDof,end)); % the last node
p_end_end = T_end_end(1:3,4);

ceq = [ceq;
    %x(:,1) - zeros(12,1)
    p_end_start - x_start
    p_end_end - x_end % == 0
%       x(1:6,end) - [pi/3, pi/2, -pi/6,pi/4,pi/2,pi/3]'
    x(7:end,end) - zeros(6,1)
    ];

end
