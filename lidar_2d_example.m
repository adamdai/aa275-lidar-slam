clear;

% Create a binary warehouse map and place obstacles at defined locations
p = zeros(100,100);
p(1:20,1:20) = ones(20,20);
p(51:100,31:50) = ones(50,20);
p(11:40,61:80) = ones(30,20);
p(71:80,61:100) = ones(10,40);
p(51:60,11:30) = ones(10,20);
map = binaryOccupancyMap(p);

% Visualize map with obstacles
figure
hold on
show(map)
grid on
daspect([1 1 1])

% % Add AGV to the map
pose = [30 60 0];
%plot(pose);

% Simulate lidar sensor. Use default detection angle of [-pi pi]
lidar = rangeSensor;
% Set min and max values of the detectable range of the sensor in meters
lidar.Range = [0 50];

% Generate lidar readings
[ranges,angles] = lidar(pose,map);
local_scan = lidarScan(ranges,angles);
global_scan = lidarScan(local_scan.Cartesian + [pose(1),pose(2)]);
%plot(global_scan)

slamObj = lidarSLAM;
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

addScan(slamObj, local_scan);
show(slamObj);