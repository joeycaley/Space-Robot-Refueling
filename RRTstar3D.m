% This function is modifed based on the example developed by 
% Omkar Halbe, Technical University of Munich, 31.10.2015

%Modified by ML for 3D RRT*
function [EndLocation,cost,iter] = RRTstar3D(start, goal, bias, MaxDist, MaxTree,enable_visual)
% clear; clc
% close all;

if nargin < 1
    start.x = 5;
    start.y = 5;
    start.z = 5;
end

if nargin < 2
    goal.x = 475;
    goal.y = 475;
    goal.z = 475;
end

if nargin < 3
    bias = 1;
end

if nargin < 4
    MaxDist = 50;    % the maximum distance allowed to travel for each time
end

if nargin < 5
    MaxTree = 10000; % the maximum number of nodes in the tree
end

if nargin < 6
    enable_visual = false;
    close all;
end

%% initialization
xMin = 0;    % minimum x value
yMin = 0;    % minimum y value
zMin = 0;   % Min z value   %%%%% New, added z min and Max

%For max values, enter near-field path planning when within 500 m?
xMax = 500;  % maximum x value
yMax = 500;  % maximum y value
zMax = 500; %Max z value



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Determine start point from
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% orbital code section
xStart = start.x;   % x coordinate of the starting point
yStart = start.y;   % y coordinate of the starting point
zStart = start.z;   % z coordinate of starting point


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Potentially pick goal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% position from orbital info
xGoal  = goal.x; % x coordinate of the goal point
yGoal  = goal.y; % y coordinate of the goal point
zGoal = goal.z; % z coordinate of goal point
thresh = 25;   % acceptable distance from the goal point


% we will use an structure array to represent the tree structure
% each entry in the array represent one node in the tree 
% (i.e., one point in the 2D configuration space)
% each node the following element:
%  - x: the x coordinate of the node
%  - y: the y coordinate of the node
%  - cost: the cost to arrive at this node
%  - parent: the index of the parent node 

% The tree will be initialized by the starting point
nodes(1).x = xStart;
nodes(1).y = yStart;
nodes(1).z = zStart;
nodes(1).cost = 0;
nodes(1).parent = 0;

% obstacles are represented by spheres polygons
% obstacles(1).xv = 40+20*cos(linspace(0,2*pi,6))';
% obstacles(1).yv = 70+20*sin(linspace(0,2*pi,6))';
% 
% obstacles(2).xv = 70+20*cos(linspace(0,2*pi,6))';
% obstacles(2).yv = 45+20*sin(linspace(0,2*pi,6))';
% 
% obstacles(3).xv = 25+15*cos(linspace(0,2*pi,5))';
% obstacles(3).yv = 25+15*sin(linspace(0,2*pi,5))';
[scale, Offset] = ObstacleGenerate(4,false);

%% visualization
if enable_visual
    view(3); hold on; grid on
    
    axis equal
    axis([0,500,0,500,0,500]);
    % plot the boundary
%     plot([0,0,500,500,0],[0,500,500,0,0],'k--','LineWidth',1);
    % plot the starting point
    plot3(xStart, yStart, zStart, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
    % plot the goal point and goal region
    plot3(xGoal, yGoal, zGoal, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Goal region left out, major changes for 3D here
    % plot the obstacles
    [xUnit,yUnit,zUnit] = sphere(20);
    for i=1:length(scale)
        surf(scale(i).*xUnit+Offset(i).x,scale(i).*yUnit+Offset(i).y,scale(i).*zUnit+Offset(i).z)
    end
end


%% search
iter = 2;
% the value of "bias" can affect how to sample from the C-space
% bias = 1: no bias
% bias < 1: biased toward the goal  (xMax, yMax)
% bias > 1: biased toward the start (xMin, yMin)
% bias = 1;%1/sqrt(2);
while iter < MaxTree   
    % sample a random point
    xRand = (xMax-xMin)*rand^bias;
    yRand = (yMax-yMin)*rand^bias;
    zRand = (zMax-zMin)*rand^bias;
    % sort the tree to find the nearest node
    p_index = sort_tree(nodes, xRand, yRand, zRand);
    xNear = nodes(p_index).x;
    yNear = nodes(p_index).y;
    zNear = nodes(p_index).z;
    % plan to motion from the nearest point to the random point by MaxDist
    [xNew, yNew, zNew] = local_planner3D(xNear,yNear,zNear,xRand,yRand,zRand,MaxDist);
    % check collision of the new path from the nearest point to the new
    % point
    c_test = collision_detector3D(xNear,yNear,zNear,xNew, yNew, zNew, scale, Offset);
    % if there is a collision, dump the current trial
    if c_test
        continue;
    end

    if iter ==2
    % update the tree    
        nodes(iter).x = xNew;       %#ok<*AGROW,*SAGROW> 
        nodes(iter).y = yNew;       % the new point as a new node
        nodes(iter).z = zNew;
        nodes(iter).parent = p_index; % the nearest point is the parent node
        nodes(iter).cost = distance3D(xNew,yNew,zNew,xNear,yNear,zNear) + nodes(p_index).cost; %parent's cost + distance
           %plotting
           if enable_visual
               plot3([nodes(iter).x; nodes(p_index).x],[nodes(iter).y; nodes(p_index).y],[nodes(iter).z; nodes(p_index).z;],'r');
               pause(0.0001);
           end
    else
    %finding nodes sufficiently near xnew (within radius of
    %15)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Could update radius of update
        nearNode = [];
        for k = 1:length(nodes)
            disTmp = distance3D(xNew,yNew,zNew,nodes(k).x,nodes(k).y,nodes(k).z);
            if disTmp <= 60 %for nodes within the radius, the total
                if collision_detector3D(xNew,yNew,zNew,nodes(k).x, nodes(k).y,nodes(k).z,scale,Offset) ~= 1 %check for collison
                    nearNode = [nearNode;k,disTmp]; %if node is within 15, store index and distance between near and new node
                end
            end
        end

    if isempty(nearNode) == 0

        %calculating the total cost of each node within radius as parents, checking collision
            costTmp = Inf*ones(1,length(nearNode));
            for j = 1:length(nearNode)
                sizeCheck = size(nearNode);
                if sizeCheck(1) == 1
                    costTmp(j) = nodes(nearNode(1)).cost + nearNode(2);
                else
                    costTmp(j) = nodes(nearNode(j,1)).cost + nearNode(j,2); %calculating total cost
                end
            end
            [~, mIndex] = min(costTmp); 
            p_index = nearNode(mIndex,1); %node index with lowest total cost is stored as parent index
        
            nodes(iter).x = xNew;
            nodes(iter).y = yNew;
            nodes(iter).z = zNew;
            nodes(iter).parent = p_index;
            nodes(iter).cost = costTmp(mIndex);
        
        %calculating the total cost of each node within radius as children
        %of new node
            for j = 1:length(nearNode)
                sizeCheck = size(nearNode);
                if sizeCheck(1) == 1
                    p_costTmp = costTmp(mIndex) + nearNode(2);
                    if p_costTmp < nodes(nearNode(1)).cost %if total cost is reduced with new node as parent node, new node replaces parent node for near node
                        nodes(nearNode(j,1)).parent = iter;
                        nodes(nearNode(j,1)).cost = p_costTmp;
                    end
                else
                    p_costTmp = costTmp(mIndex) + nearNode(j,2); %adding stored new node cost to stored distance between near node and new node
                    if p_costTmp < nodes(nearNode(j,1)).cost  %if total cost is reduced with new node as parent node, new node replaces parent node for near node
                        nodes(nearNode(j,1)).parent = iter;
                        nodes(nearNode(j,1)).cost = p_costTmp;
                    end
                end
            end
            %plot the new path
            if enable_visual
                plot3([nodes(iter).x; nodes(p_index).x],[nodes(iter).y; nodes(p_index).y],[nodes(iter).z; nodes(p_index).z], 'r');
                pause(0.001);
            end
    end
    end

    
    % check if it reaches the goal region
    if distance3D(xNew, yNew,zNew, xGoal, yGoal, zGoal) <= thresh
        break
    end
    
    iter = iter + 1;
end

%% final cost
cost = nodes(end).cost;
%% determine the feasible path

if iter < MaxTree
    xPath(1) = xGoal;
    yPath(1) = yGoal;
    zPath(1) = zGoal;
    xPath(2) = nodes(end).x;
    yPath(2) = nodes(end).y;
    zPath(2) = nodes(end).z;
    
    
    parent = nodes(end).parent;
    j=0;
    while 1
        xPath(j+3) = nodes(parent).x;
        yPath(j+3) = nodes(parent).y;    
        zPath(j+3) = nodes(parent).z;
        parent = nodes(parent).parent;
        if parent == 0
            break
        end
        j=j+1;
    end
    if enable_visual
        plot3(xPath, yPath, zPath, 'g', 'Linewidth', 3);
    end
else
    disp('No path found. Increase number of iterations and retry.');
    cost = NaN;
    iter = NaN;
end

EndLocation = nodes(end);
%% updated plotting for RRT*
if true
    figure
    view(3) 
    hold on; grid on
    axis equal
    axis([0,500,0,500,0,500]);
    % plot the boundary
    %plot([0,0,100,100,0],[0,100,100,0,0],'k--','LineWidth',1);
    % plot the starting point
    plot3(xStart, yStart, zStart, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
    % plot the goal point and goal region
    plot3(xGoal, yGoal,zGoal, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
    
    [xUnit,yUnit,zUnit] = sphere(20); %unit sphere for obstacles and region
    plot3(thresh*xUnit+xGoal,thresh*yUnit+yGoal,thresh*zUnit+zGoal);
    %th = 0:pi/50:2*pi;
    %xcircle = thresh * cos(th) + xGoal;
    %ycircle = thresh * sin(th) + yGoal;
    %h = plot(xcircle, ycircle);
    % plot the obstacles
    
    for i=1:length(scale)
        surf(scale(i).*xUnit+Offset(i).x,scale(i).*yUnit+Offset(i).y,scale(i).*zUnit+Offset(i).z)
    end

    %plot each node and connection
    for i = 2:length(nodes)
    plot3([nodes(i).x; nodes(nodes(i).parent).x],[nodes(i).y; nodes(nodes(i).parent).y],[nodes(i).z; nodes(nodes(i).parent).z], 'b');
                pause(0.001);
    end

    %plot final path
    plot3(xPath, yPath, zPath, 'g', 'Linewidth', 3);
end
