% Mobile Robot Project

% Clear the command window and workspace, and close all open figures
clear, clc,close all;

% Prompts the user to select an image file and retrieves the file's path and name
[filename, pathname] = uigetfile({'*.jpg;*.png;*.bmp;*.gif', 'Image Files (*.jpg, *.png, *.bmp, *.gif)'}, 'Select an image file');

% Check if the user canceled the file selection
if isequal(filename, 0) || isequal(pathname, 0)
    % Display a message indicating the user canceled the operation
    disp('User canceled the operation');
else
    % Read the selected image using the obtained file path and name
    fullFilePath = fullfile(pathname, filename);
    img = imread(fullFilePath);
    
    % Display the imported image in a subplot
    subplot(1, 2, 1)
    imshow(img);
    title('Imported Image')
end 

% Pause the execution for 3 seconds (allows time for image viewing)
pause(3)
% -----------------------------------------------------------------------------------------------------------------------
% This ratio must be multiplied to all dimensions in the image (Borders & obstacles)
% Defines variables to represent real room dimensions and corresponding image dimensions
% Calculates conversion ratios from centimeters to pixels for length and width

RoomReallength = 250;               % Real length of the room (in centimeters)
RoomImagelength = size(img, 1);     % Image length obtained from the imported image
RoomRealWidth = 350;                % Real width of the room (in centimeters)
RoomImageWidth = size(img, 2);      % Image width obtained from the imported image
cm2pixel_Lratio = RoomReallength / RoomImagelength;  % Conversion ratio for length (cm to pixels)
cm2pixel_Wratio = RoomRealWidth / RoomImageWidth;    % Conversion ratio for width (cm to pixels)
% -----------------------------------------------------------------------------------------------------------------------
% Convert the RGB image to grayscale
grayImg = rgb2gray(img);   % Convert the imported image to grayscale
subplot(1, 2, 2)
imshow(grayImg);           % Display the grayscale image in a subplot
title('gray Image')        % Set title for the displayed image
pause(3)                   % Pause the execution for 3 seconds
% -----------------------------------------------------------------------------------------------------------------------
% Detect edges using the Canny edge detection algorithm
% The threshold value of 0.6754372 sets the sensitivity for edge detection
edges = edge(grayImg, 'canny', 0.6754372);
figure;
subplot(1, 2, 1)
imshow(edges);             % Display the detected edges in a subplot
title('edges image')       % Set title for the displayed edges
pause(3)                   % Pause the execution for 3 seconds
% -----------------------------------------------------------------------------------------------------------------------
% Perform dilation to strengthen edges (zeros) using the structure element
se = strel('disk', 2);     % Define a disk-shaped structuring element for dilation
edges_dilated = imdilate(edges, se);  % Perform dilation on the detected edges
subplot(1, 2, 2)
imshow(edges_dilated)      % Display the dilated edges in a subplot
title('dilated image')     % Set title for the displayed dilated edges
pause(3)                   % Pause the execution for 3 seconds
% -----------------------------------------------------------------------------------------------------------------------
% Invert the edges to make black rectangles white for detection
inverted_edges = ~edges_dilated;  % Invert the dilated edges
figure;
subplot(1, 2, 1)
imshow(inverted_edges)     % Display the inverted edges in a subplot
title('inverted edges')    % Set title for the displayed inverted edges
pause(3)                   % Pause the execution for 3 seconds
% -----------------------------------------------------------------------------------------------------------------------
% Use regionprops to measure properties of the black regions [xLeft, yTop, width, height]
stats = regionprops(inverted_edges, 'BoundingBox');  % Measure properties of the black regions
% -----------------------------------------------------------------------------------------------------------------------
% Initialize an array to store region properties filtered by size criteria
filteredStats = [];  % To store region properties that match the size criteria (without errors)

% Filter regions with width or height larger than 10 and store in filteredStats
for i = 1:numel(stats)
    % Check if width and height are both greater than 10
    if stats(i).BoundingBox(3) > 10 & stats(i).BoundingBox(4) > 10
        % Store the region properties satisfying the condition
        filteredStats = [filteredStats; stats(i)];
    end
end
% -----------------------------------------------------------------------------------------------------------------------
% Extract obstacle dimensions from filteredStats
a = 0;  % Initializing counter for obstacles
b = 0;  % Initializing counter for bounding box properties

for i = 1:numel(filteredStats)
    if RoomImagelength - filteredStats(i).BoundingBox(4) > 20
        a = a + 1;  % Incrementing obstacle counter
        b = 0;      % Resetting bounding box property counter for each obstacle
        
        for j = 1:numel(filteredStats(1).BoundingBox)
            b = b + 1;  % Incrementing bounding box property counter
            % Store obstacle dimensions in the 'obstacles' struct
            obstacles(a).BoundingBox(b) = filteredStats(i).BoundingBox(j);
        end
    end
end
% -----------------------------------------------------------------------------------------------------------------------
% Create a mask matching the original image size for rectangle drawing
mask = zeros(size(inverted_edges, 1), size(inverted_edges, 2));
filled_image = inverted_edges;  % Initializing filled_image as a copy of inverted_edges

% Loop through obstacles, draw rectangles on the mask, and fill them in the image
subplot(1, 2, 2);

for i = 1:numel(obstacles)
    x = round(obstacles(i).BoundingBox(1));      % Get X-coordinate of the obstacle
    y = round(obstacles(i).BoundingBox(2));      % Get Y-coordinate of the obstacle
    width = round(obstacles(i).BoundingBox(3));  % Get width of the obstacle
    height = round(obstacles(i).BoundingBox(4)); % Get height of the obstacle

    % Draw filled rectangles on the mask, excluding the largest one
    mask(y:y+height-1, x:x+width-1) = 1;

    % Apply the mask to filled_image to fill detected rectangles with zeros
    filled_image(mask == 1) = 0;  % Filling detected areas with black color
    % Display the filled image with obstacles
    imshow(filled_image);
    pause(0.5);  % Pause to visualize each step
end

title('Filled Obstacles Image');  % Title for the final image with filled obstacles

% Add a grid to the image and allow user input by clicking on the image
hold on;      % Retains the current plot and adds new elements to it
grid on;      % Displays the gridlines on the current plot
% -----------------------------------------------------------------------------------------------------------------------
[targetx, targety] = ginput(1);  % Capture a single mouse click to specify the goal point
disp(['Target (x, y): ', num2str(x), ', ', num2str(y)]);  % Display the target coordinates

% Display the clicked coordinates on the image as text in red color
text(targetx, targety, ['(', num2str(x), ', ', num2str(y), ')'], 'Color', 'red', 'FontSize', 10);
% -----------------------------------------------------------------------------------------------------------------------
% Specify the Excel file name (The Data from the Arduino)
filename = 'G:\Faculty\2nd EE - 1st term\Matlab\Projects\Mobile Robot Project\Data.xlsx';  % Replace with the actual file path

% Read the data from the Excel file
[Data, textData, raw] = xlsread(filename);

% Extracting the angles & obstacle distances from the Data
angle = Data(:,1)';     % Extract angles from the first column of the Data
obDistance = Data(:,2)';  % Extract obstacle distances from the second column of the Data
% -----------------------------------------------------------------------------------------------------------------------
% Data correction...
% The following code aims to read data related to angles and obstacle distances from the Excel file
% and corrects any outliers or inconsistencies in the obstacle distance data
% by applying a median-based correction mechanism for subsets of data within the set of obstacle distances.

distance = zeros(1, 5);  % Initialize an array to hold 5 distances
counter = 1;

for l = 1:size(obDistance, 2) / 5  % Divide the dataset into subsets of 5 elements
    counter = counter + 5;  % Increment the counter by 5
    j = counter - 6;  % Adjust index for data extraction
    n = 5;  % Define the subset size
    i = 0;  % Initialize a counter

    % Extract 5 distances for calculation
    for k = 1:n
        i = i + 1;  % Increment the inner loop counter
        j = j + 1;  % Increment the index for data extraction
        distance(i) = obDistance(j);  % Store the extracted distance
    end

    % Calculate median, quartiles, IQR, and define a threshold for outliers
    median_distance = median(distance);  % Calculate the median of the distances
    Q1 = prctile(distance, 25);  % Compute the first quartile
    Q3 = prctile(distance, 75);  % Compute the third quartile
    IQR = Q3 - Q1;  % Calculate the interquartile range (IQR)

    % Define a threshold for identifying outliers
    threshold = 1.5 * IQR;

    % Identify outliers in the subset
    outliers = (distance < (Q1 - threshold)) | (distance > (Q3 + threshold));

    % Replace outliers with the median of non-outliers in the subset
    distance(outliers) = median_distance;
    obDistance(1, counter - 5:counter - 1) = distance;  % Update the original dataset with corrected values
end
% -----------------------------------------------------------------------------------------------------------------------
% The following code compares the calculated width and height of obstacles based on ultrasonic sensor data
% The provided data records the distances as the robot reads angles from 0 to 180 degrees
% For instance, the data stream might look like: 22 20 21 23 20 21 23 20 50 55 54 55 53 52 51 52 80 88 85 86 ...
% The algorithm identifies subsets within the data with consistent trends
% These subsets indicate stable obstacle readings despite changing distances
% Using these subsets, the code applies the cosine law to calculate the lengths of opposite obstacles
% It filters out obstacle lengths that don't conform to the unique trend, retaining representative distances
% The resulting 'L3' variable stores obstacle lengths representative of the actual obstacles detected

% Loop through consecutive obstacle distance data to calculate differences
for i=1:size(obDistance,2)-1
    Diff(i)= abs(obDistance(i)-obDistance(i+1)); % Calculate absolute differences between consecutive obstacle distances
end

temp= Diff>10; % Identify trends where differences exceed 10 units
angle=zeros(1,sum(temp)+2); % Initialize an array to store angle indices
angle(1)=0; % Set the first angle as 0
angle(end)=length(obDistance); % Set the last angle as the length of obstacle distance
angle(2:end-1) = find(temp == 1); % Identify indices where differences indicate a new trend in obstacle distances
obsNum= length(angle)-1; % Calculate the number of unique trend subsets

% Calculate lengths of opposite obstacles using cosine law based on unique trends
for i=1:obsNum
    index1(i)=angle(i)+1; % Obtain indices for the starting point of each subset
    L1(i)=obDistance(index1(i)); % Store obstacle distances at the starting point of each subset
    index2(i)=angle(i+1); % Obtain indices for the ending point of each subset
    L2(i)=obDistance(index2(i)); % Store obstacle distances at the ending point of each subset
    L3(i)=sqrt((L1(i))^2+(L2(i))^2-2*L1(i)*L2(i)*cosd(index2(i)-index1(i))); % Calculate obstacle length using cosine law
end

L3=L3*1.015; % Scale the calculated obstacle lengths by a factor of 1.015 To address small errors in calculating distances
% Filter obstacle lengths to keep representative distances
representativeDistances = L3(L3 >= 20); % Filter obstacle lengths greater than or equal to 20 units (Getting the obstacles with dimentions larger than 20 cm)
% -----------------------------------------------------------------------------------------------------------------------
width_index = 0;             % Initialize the index for width detection
length_index = 0;            % Initialize the index for length detection

% Calculate real dimensions of obstacles using pixel ratios
for i = 1:length(obstacles)
    realObstaclesDim(i).BoundingBox(1) = cm2pixel_Lratio * obstacles(i).BoundingBox(4);
    realObstaclesDim(i).BoundingBox(2) = cm2pixel_Wratio * obstacles(i).BoundingBox(3);
end

% Find the indices matching the representative distances in the real obstacle dimensions
for i = 1:length(realObstaclesDim)
    for j = 1:length(realObstaclesDim(1).BoundingBox)
        if (abs(representativeDistances(j) - realObstaclesDim(i).BoundingBox(2))) <= 1
            width_index = i;     % Assign index for width if a match is found
        elseif (abs(representativeDistances(j) - realObstaclesDim(i).BoundingBox(1))) <= 1
            length_index = i;    % Assign index for length if a match is found
        end
    end
end
% -----------------------------------------------------------------------------------------------------------------------
% Determine the robot's starting position based on the matched obstacle in the map
if length_index > 0
    % If a matching length of an obstacle is found, the robot starts slightly
    % beyond the detected obstacle's right edge and aligns with the top edge.
    startPosex = obstacles(length_index).BoundingBox(1) + obstacles(length_index).BoundingBox(3) + 10;
    startPosey = obstacles(length_index).BoundingBox(2);
elseif width_index > 0
    % If a matching width of an obstacle is found, the robot starts at the detected
    % obstacle's left edge and positions slightly below the bottom edge.
    startPosex = obstacles(width_index).BoundingBox(1);
    startPosey = obstacles(width_index).BoundingBox(2) + obstacles(width_index).BoundingBox(4) + 10;
end
% -----------------------------------------------------------------------------------------------------------------------
% Main code for the RRT (Rapidly-exploring Random Tree) algorithm

% Parameter initialization
x_I = startPosex; y_I = startPosey;       % Set the initial point
x_G = targetx; y_G = targety;        % Set the goal point
disp(['Start position (x, y): ', num2str(startPosex), ', ', num2str(startPosey)]);
Thr = 20;                   % Set the threshold for the goal point
Delta = 20;                 % Set the extension step length

% Tree initialization
T.v(1).x = x_I;             % T represents the tree, v represents the nodes. Include the start point in T.
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;         % The starting node's parent is itself
T.v(1).yPrev = y_I;
T.v(1).dist = 0;            % Distance from the parent node to this node, here using Euclidean distance
T.v(1).indPrev = 0;         % Index of the parent node

Imp = grayImg;               % Assign the gray image to Imp variable
figure;                      % Create a new figure
imshow(filled_image)         % Display the filled_image
title('RRT Algorithm')       % Set title of the figure to 'RRT Algorithm'
xL = size(Imp, 1);           % Obtain the map's x-axis length
yL = size(Imp, 2);           % Obtain the map's y-axis length
hold on                      % Keep the existing plot while adding new elements
plot(x_I, y_I, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');  % Plot the initial point (start position)
plot(x_G, y_G, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % Plot the goal point


count = 1;  % Initialize the counter to track the number of iterations
for iter = 1:3000  % Iterate through 3000 cycles for the RRT algorithm
    x_rand = [rand * 800, rand * 800];	% Generate a random point (x, y) within the range of 0 to 800

    % Initialize minimum distance and its index to the first node in the tree T
    minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2);
    minIndex = 1;

    % Loop through the tree nodes to find the nearest node to the random point x_rand
    for i = 2:size(T.v, 2)
        distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2);
        % Check if the current node is closer to x_rand than the previous nearest node
        if(distance < minDis)
            minDis = distance;
            minIndex = i;   
        end     
    end
    % Store the x and y coordinates of the nearest node in x_near
    x_near(1) = T.v(minIndex).x;   
    x_near(2) = T.v(minIndex).y;
    
    % Calculate new coordinates x_new based on x_near and a random direction
    theta = atan2((x_rand(2) - x_near(2)), (x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    
    % Check for collision between x_near and x_new with the obstacles (Imp)
    if ~collisionChecking(x_near, x_new, Imp) 
        continue; % Skip current iteration if collision occurs
    end
    
    count = count + 1;  % Increment the count for the new node in the tree
    
    % Add x_new to the tree T as a new node with its parent, distance, and index
    T.v(count).x = x_new(1);          
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = Delta;
    T.v(count).indPrev = minIndex;    
    
    % Check if x_new is close enough to the goal point (x_G, y_G)
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < Thr)
        break % Exit the loop if the goal is reached within threshold distance (Thr)
    end
   
    % Plot the path between x_near and x_new as well as the new node x_new
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'b', 'Linewidth', 2);
    plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
   
    pause(0.1);  % Pause the execution for visualization
end

if iter < 2000  % Check if the iteration count is less than 2000 (indicating a path is found)
    % Assign the goal point and the last node's coordinates to the path
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % Get the index of the previous node to traverse the path
    j = 0;
    % Traverse through the tree nodes to construct the complete path
    while 1
        path.pos(j + 3).x = T.v(pathIndex).x;
        path.pos(j + 3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev; % Move to the previous node
        if pathIndex == 1
            break % Exit the loop when reaching the start node
        end
        j = j + 1;
    end
    path.pos(end + 1).x = x_I; path.pos(end).y = y_I; % Include the start point in the path
    % Plot the calculated path on the map using green lines
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j - 1).x], [path.pos(j).y; path.pos(j - 1).y], 'g', 'Linewidth', 4);
    end
else
    disp('Error, no path found!'); % Display an error message if no path is found
end
% -----------------------------------------------------------------------------------------------------------------------
% Functions...

% The purpose of the following function is to determine the feasibility of a trajectory between two given poses (startPose and goalPose) within a provided map.
% It evaluates whether the trajectory is obstructed by obstacles present in the map.

function feasible = collisionChecking(startPose, goalPose, map)
    feasible = true; % Initialize feasibility as true by default    
    dir = atan2(goalPose(2) - startPose(2), goalPose(1) - startPose(1)); % Calculate the direction of movement
    
    % Iterate through points between startPose and goalPose to check for collisions
    for r = 0:0.5:sqrt(sum((startPose - goalPose).^2))
        posCheck = startPose + r .* [cos(dir), sin(dir)]; % Calculate the positions to be checked
        
        % Check if the points are feasible (within map boundaries and not obstructed)
        if ~(feasiblePoint(ceil(posCheck), map) && feasiblePoint(floor(posCheck), map) ...
            && feasiblePoint([ceil(posCheck(1)), floor(posCheck(2))], map) ...
            && feasiblePoint([floor(posCheck(1)), ceil(posCheck(2))], map))
            feasible = false; % If any point is infeasible, set feasibility to false
            break; % Exit the loop
        end
    end

    % Check if the goalPose is feasible (within map boundaries and not obstructed)
    if ~(feasiblePoint(ceil(goalPose), map) && feasiblePoint(floor(goalPose), map) ...
        && feasiblePoint([ceil(goalPose(1)), floor(goalPose(2))], map) ...
        && feasiblePoint([floor(goalPose(1)), ceil(goalPose(2))], map))
        feasible = false; % If the goalPose is infeasible, set feasibility to false
    end
end
% -----------------------------------------------------------------------------------------------------------------------
% The following function checks if the provided point lies within the boundaries of the map
% and whether it corresponds to an obstacle or free space in the map.

function feasible = feasiblePoint(point, map)
    feasible = true; % Initialize feasibility as true by default
    
    % Check if the point is within the map boundaries and not an obstacle (value 255)
    if ~(point(1) >= 1 && point(1) <= size(map, 2) && point(2) >= 1 ...
        && point(2) <= size(map, 1) && map(point(2), point(1)) == 255)
        feasible = false; % If the point is out of boundaries or an obstacle, set feasibility to false
    end
end
