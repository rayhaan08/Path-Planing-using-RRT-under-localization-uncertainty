%% Nearest neighbor
function [near_node, near_node_idx, found, dist_min] = nearest_neighbor(pt_new,  tree, obstacles, connection_matrix)

method = 0;
dist_min = 10000;	% Random Large minimum distance
found = 0; near_node=[]; near_node_idx=[]; 
for i = 1:size(tree,1)

    % Check if the connection has no obstacles
    if collision_check_segment(pt_new(1,1),pt_new(1,2), tree(i,1), tree(i,2), obstacles) == 1
    elseif collision_check_point(pt_new(1,1),pt_new(1,2), obstacles) == 1
        
        
    else
        found = 1;
        % Calculates the path distance
        
        if method == 1
            
                    idx_cur = i;
                    %path_dist = path_dist(path, Connection, idx_cur)
        
                    dist = 0;%idx_cur =
                    idx_n = 2;
                    while idx_n>1
        
                        [~,idx_n] = find(connection_matrix(idx_cur,:) == 1);
        
                        dist = dist + pdist([tree(idx_n,:); tree(idx_cur,:)],'euclidean');
        
                        idx_cur = idx_n;
                    end
        
                    if isempty(dist)
                        path_dist=0;
                    else
                        path_dist = dist;
                    end
                    node_dist = pdist([tree(i,1),tree(i,2);pt_new(1,1),pt_new(1,2)],'euclidean');
                    tot_dist = path_dist + node_dist;
                    if tot_dist <dist_min
                        near_node = tree(i,:);
                        dist_min = tot_dist;
                        near_node_idx = i;
                    end
                    
                    
        elseif method == 0
            node_dist = pdist([tree(i,1),tree(i,2);pt_new(1,1),pt_new(1,2)],'euclidean');
            if node_dist <dist_min
                near_node = tree(i,:);
                dist_min = node_dist;
                near_node_idx = i;
            end
        end
        
    end
end
