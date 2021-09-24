clc;clear all;close all;

% Define Number of max Iterations
itr_max=10;

min_len =1000; min_path=[]; min_itr=0;	% Initializing variables
for itr = 1:itr_max
    
	close all	%Supress figures

    [start_state, obstacles, goal_region] = generate_obstacles;
    start_state = [5,50];
    obstacles = [5,10,15,10,15,20;10,40,20,40,20,50;20,70,30,70,30,80;30,20,40,20,40,30;40,50,50,50,50,60;50,5,60,5,60,15;55,80,65,80,65,90;60,40,70,40,70,50;70,20,80,20,80,30;75,65,85,65,85,75];
    goal_region = [90,0,100,0,100,100,90,100];
    hold on
    
    % Call the RRT code to generate samples
    [path, path_length] = RRT(start_state, goal_region, obstacles);
    

    % Check the shortest path length iteration
    if path_length < min_len
        min_len = path_length;
        min_path = path;
        min_itr = itr;
    end
end

% Display the iteration with shortest path length
generate_obstacles;
hold on
plot(min_path(:,1), min_path(:,2))
xlim([0,100])
ylim([0,100])
str_fig = "Shortest Iteration: Itr No " + int2str(min_itr);
title(str_fig)
