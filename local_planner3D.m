function [xNew, yNew, zNew] = local_planner3D(xNear,yNear,zNear,xRand,yRand,zRand,MaxDist)

    dist = sqrt((xRand-xNear).^2 +(yRand-yNear).^2 + (zRand-zNear).^2);
    % if the distance to go to the (xRand,yRand) is greater than the
    % maximum distance, then travel MaxDist toward (xRand,yRand)
    if dist > MaxDist
        xNew = xNear + MaxDist*(xRand-xNear)/dist;
        yNew = yNear + MaxDist*(yRand-yNear)/dist;
        zNew = zNear + MaxDist*(zRand-zNear)/dist;
    else % otherwise, travel to the (xRand,yRand)
        xNew = xRand;
        yNew = yRand;
        zNew = zRand;
    end
end