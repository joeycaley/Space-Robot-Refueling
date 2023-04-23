clear
clc
close all

%This portion of the code takes the location of the spcaecraft from the
%orbital mechanics, and calculates a trajectory for the spacecraft to get
%within 25 m of the target craft

% Replace with input data from orbital mechanics
T = 1
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

goal.x = EndLoc.x - goal.x;
goal.y = EndLoc.y - goal.y;
goal.z = EndLoc.z - goal.z;

