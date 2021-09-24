
function [path, path_length] = RRT(start_state, goal_region, obstacles)
xi = start_state(1);
yi = start_state(2);
tree = start_state;
flag = 0;
e=2;

% Creating a connection matrix to show the relation between nodes. Initializing 5x5 randomly, size increases with nodes
connection_matrix = zeros(5,5);
itr=0;
while flag==0
    
    itr = itr+1;
    
    %% Sampling points only in the workspace
    x_lb = 0;
    x_ub = 100;
    xnew = (x_ub-x_lb).*rand + x_lb;    
    y_lb = 0;
    y_ub = 100;
    ynew = (y_ub-y_lb).*rand + y_lb;
    
    pt_new = [xnew,ynew];
    
    %% Nearest neighbor
    [near_node, near_node_idx, found, dist_min] = nearest_neighbor(pt_new,  tree, obstacles, connection_matrix);            
    
    
    if found==1		% Checking if a valid nearest node was found
        if dist_min>e	% Checking if distance is less than step size
            
            pt_new(1,1) = (e*pt_new(1,1) + (dist_min-e)*near_node(1,1))/(dist_min);
            pt_new(1,2) = (e*pt_new(1,2) + (dist_min-e)*near_node(1,2))/(dist_min);
            
        end
        
        plot([near_node(1,1), pt_new(1,1)], [near_node(1,2), pt_new(1,2)])
        
	% Update the tree and connection matrix
        tree = [tree;pt_new];
        connection_matrix(size(tree,1),near_node_idx) = 1;
        
	% Check if goal was reached
        if pt_new(1,1) >= 90
            flag=1;
        end
        
    end
    
end

%% Calculates the path and path distance
idx_cur = size(tree,1);
path_length = 0;path =[];
idx_n = 2;	%choosing a random number greater than 2
while idx_n>1
    
	% Decode the connection matrix by finding connected node
    [~,idx_n] = find(connection_matrix(idx_cur,:) == 1);
    
    path_length = path_length + pdist([tree(idx_n,:); tree(idx_cur,:)],'euclidean');
    path = [tree(idx_cur,:); path]
    idx_cur = idx_n;
end
path = [start_state;path];
