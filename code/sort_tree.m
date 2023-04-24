function index = sort_tree(nodes, xRand, yRand, zRand)

    dist = Inf*ones(1,length(nodes));
    for j = 1:length(nodes)
        %dist(j) = distance(xRand,yRand,zRand,nodes(j).x,nodes(j).y,nodes(j).z);
        dist(j) = sqrt((xRand-nodes(j).x).^2 +(yRand-nodes(j).y).^2 + (zRand-nodes(j).z).^2);
    end
    [~, index] = min(dist);
end

