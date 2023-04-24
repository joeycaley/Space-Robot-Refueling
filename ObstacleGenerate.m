function [scale, Offset] = ObstacleGenerate(ObstacleNum,enable_vis)

if nargin < 1
    ObstacleNum = 3; %3 obstacles by default
end

if nargin < 2
    enable_vis = true;
    close all;
end

hold on
view(3)
grid on
box on
for i= 1:ObstacleNum
    scale(i) = randi([20,85]);
    [x,y,z] = sphere(20);
    x = scale(i).*x;
    y = scale(i).*y;
    z = scale(i).*z;
    Offset(i).x = randi([70,400]);
    Offset(i).y = randi([70,400]);
    Offset(i).z = randi([70,400]);
    
    if enable_vis
        surf(x+Offset(i).x,y+Offset(i).y,z+Offset(i).z)
    end
end

if enable_vis
    axis('equal')
end

end