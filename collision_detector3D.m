function  c_test = collision_detector3D(xNear,yNear,zNear,xNew, yNew,zNew,scale, Offset)
    % we test 10 uniformally distributed points along the line that
    % connects (xNear,yNear) to (xNew,yNew)
    s = 0:0.01:1;
    xq = xNear + s.*(xNew-xNear);
    yq = yNear + s.*(yNew-yNear);
    zq = zNear + s.*(zNew - zNear);
    
    c_test = 0; % initialize as false (no collision)
    for i=1:length(scale)
        % check if any point is within the spheres by checking if the distance
        % from the sphere center to the line is <= sphere radius
        for j=1:length(xq)
            DistFromCent = sqrt((xq(j)-Offset(i).x).^2+(yq(j)-Offset(i).y).^2+(zq(j)-Offset(i).z).^2);

            if DistFromCent <= scale(i)/2+10 % if any one of the points are within radius of the sphere 
                c_test = 1; % set it true (collision)
                break;      % break the loop, no need to test other areas
            end
        end
    end
    
end