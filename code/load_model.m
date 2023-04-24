function model = load_model()
% This function load the model parameters from the URDF file and construct
% M, Slist, Mlist, Glist matrices

[links, joints] = load_urdf('ur5.urdf');

%% Construction of M and Slist
% We will compute the homogeneous transformation matrix from the base frame
% to each joint frame at the zero position, and assign it to a new field 
% "T" for each joint structure.
for i=1:length(joints)
    % first construct the homogeneous transformation matrix from the parent
    % joint frame (i-1) to current joint frame (i)
    T_i = RpToTrans(joints(i).R,joints(i).Offset');
    if i==1 % if it is the base joint
        joints(i).T0 = T_i;
    else    % otherwise
        joints(i).T0 = joints(i-1).T0*T_i; % we compute them recursively
    end
end

% The end-effector frame (the last "joint") configuration at the zero position
M = joints(end).T0;

% We first formulate the screw axis in each joint frame, and then use
% adjoint matrix to compute the screw axes in the space frame
Slist = zeros(6,length(joints)-2);
for i=2:length(joints)-1 % the first and last entries are not joint (fixed)
    AdT = Adjoint(joints(i).T0); % compute the adjoint matrix
    if strcmpi(joints(i).Type,"continuous") % revolute joint
        B_i = [joints(i).Axis'; zeros(3,1)]; % screw axis in the joint frame
    elseif strcmpi (joints(i).Type,"prismatic") % prismatic joint
        B_i = [zeros(3,1); joints(i).Axis']; % screw axis in the joint frame
    end
    Slist(:,i-1) = AdT*B_i;
end


links(1).M_i = eye(4); % the base link is at the fixed coordinate
for i=2:length(links)
    % first construct the homogeneous transformation matrix from the parent
    % joint frame (Ti) to the (child) link frame (Mi)
    T_M_i = RpToTrans(links(i).R,links(i).Offset');
    %     joints(i).T0*T_M_i;
    links(i).M_i = joints(i).T0*T_M_i; % we compute them recursively
end
% construct the Mlist, which is the transformation matrices from a link
% (i-1)'s to its child link (i)
Mlist = zeros(4,4,length(links));
for i=2:length(links)
    Mlist(:,:,i-1) = links(i-1).M_i \ links(i).M_i;  
end
% the last entry of the Mlist is the from the last link frame to the end effector frame
Mlist(:,:,7) = links(7).M_i \ M;



% construct the Glist, which are the spatial inertia matrices of all links
% at the link body frame
Glist = zeros(6,6,length(links) - 1);
for i=2:length(links)
    Glist(:,:,i-1) = blkdiag(links(i).Inertia, links(i).Mass*eye(3)); 
end


model.nDof = size(Slist,2);
model.nInputs = model.nDof;
model.M = M;
model.Slist = Slist;
model.Mlist = Mlist;
model.Glist = Glist;
model.joints = joints;
model.links = links;
end

