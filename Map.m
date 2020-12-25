%% 2D Map Of The Real World Environment

% This file generates a map of a selected real world environemnt. This is
% the map which is used to plan the path for robot (configuration :
% differential drive)

%% 1. Introduction
%This code segement generates the map of a residence. This map is used by the
%main section to plan the path given specific start and goal positions.

%There are 2 sections of code exeecuted by the program to generate
%the complete map of the environment.

%   Generate Boundaries for the Environment.

        % Set limits of each induvidual boundary
        % Get the total number of elements of the line boundary 
        % Reshape the x and y coordinates to match the map as per requirements
        % Concatinate induvadual boundary to replicate the real world environment

%   Set Occupancy Grid of the Map.
    
        % This section sets the occupancy grid of the real world environment. This
        % section converts the line generated to make the program understand which
        % grid particales are 100% OCCUPIED and which grid elements are 100% NOT
        % OCCUPIED

%% 1. Generate Boundaries for the Environment.
%% 1.1 Set the limits of each induvidual boundary

b = 0:0.1:33;       
ha1 = 0:0.1:7.5;   
ha2 = 12:0.1:30;
d1 = 10:0.1:20;
d2 = 0:0.1:2.5;
d3 = 5:0.1:7.5;
b1 = 15:0.1:22.5;
b2 = 25:0.1:30;
b3 = 20:0.1:25;
b4 = 15:0.1:30;
w1 = 7.5:0.1:12;
k1 = 20:0.1:25;
k2 = 25:0.1:33;
w2 = 25:0.1:27.5;
w3 = 29.5:0.1:33;

%% 1.2 Get the nubmer of elememnts of the line boundary

nb = numel(b);
nx1 = numel(ha1);
nx2 = numel(ha2);
ny1 = numel(d1);
nx3 = numel(d2);
nx4 = numel(d3);
nx5 = numel(b1);
nx6 = numel(b2);
ny2 = numel(k1);
ny3 = numel(b3);
nx7 = numel(b4);
nx8 = numel(w1);
ny4 = numel(k2);
ny5 = numel(w2);
ny6 = numel(w3);

n_x = nx1 + nx2 + nx3 +nx4+ nx5+ nx6 + 2* nb + nx7 + nx8;
n_y = ny1 + 2* nb + ny2+ ny3 + ny4 + ny5 + ny6;

n_tot = n_x + n_y; 

%% 1.3. Reshape the x and y coordinates to match the map as per requirements

bx_1 = reshape(b, [nb,1]); 
bx_2 = reshape(b, [nb,1]);
bx_3 = zeros(nb,1);
bx_4 = 30*ones(nb,1);
x_1 = reshape(ha1, [nx1,1]);
x_2 = reshape(ha2, [nx2,1]);
x_3 = 12*ones(ny1, 1);
x_4 = reshape(d2, [nx3,1]);
x_5 = reshape(d3, [nx4,1]);
x_6 = reshape(b1, [nx5,1]);
x_7 = reshape(b2, [nx6,1]);
x_8 = 7.5*ones(ny2,1);
x_9 = 15*ones(ny3,1);
x_10 = reshape(b4, [nx7,1]);
x_11 = reshape(w1, [nx8,1]);
x_12 = 7.5*ones(ny4,1);
x_13 = 12*ones(ny5,1);
x_14 = 12*ones(ny6,1);

by_1 = zeros(nb,1);
by_2 = 33*ones(nb,1);
by_3 = reshape(b, [nb,1]);
by_4 = reshape(b, [nb,1]);
y_1 = 10*ones(nx1, 1);
y_2 = 10*ones(nx2, 1);
y_3 = reshape(d1, [ny1,1]);
y_4 = 20*ones(nx3, 1);
y_5 = 20*ones(nx4, 1);
y_6 = 20*ones(nx5, 1);
y_7 = 20*ones(nx6, 1);
y_8 = reshape(k1,[ny2,1]);
y_9 = reshape(b3,[ny3,1]);
y_10 = 25*ones(nx7,1);
y_11 = 25*ones(nx8,1);
y_12 = reshape(k2, [ny4,1]);
y_13 = reshape(w2, [ny5,1]);
y_14 = reshape(w3, [ny6,1]);

%% 1.4 Concatinate each induvidual boundary to replicate the real world environment

x = [bx_1; bx_2; bx_3; bx_4; x_1; x_2; x_3; x_4; x_5; x_6; x_7; x_8; x_9; x_10; x_11; x_12; x_13; x_14];
y = [by_1; by_2; by_3; by_4; y_1; y_2; y_3; y_4; y_5; y_6; y_7; y_8; y_9; y_10; y_11; y_12; y_13; y_14];

%% 3. Set Occupancy Grid of the Map

%%Create the map variable with apporopriate boundaries and resolution
map = binaryOccupancyMap(30,33,5);

%%Set the occupancy grid of the map
setOccupancy(map, [x y], ones(n_tot, 1));

%Display the map of the generated real world environemnt.
figure
show(map)
