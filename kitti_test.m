close all;
clear all;

%% Process Kitti Velodyne Data
addpath('velodyne_points');
addpath('velodyne_points\data_pcd');

% pcloud = pcread('0.pcd');
% pcshow(pcloud);

% read pcd data into pointcloud objects
dataDir = 'velodyne_points\data_pcd';
files = dir(fullfile(dataDir, '*.pcd'));
for k = 1:length(files)
    filename = files(k).name;
    pcloud = pcread(filename);
	PointCloud(k,1) = pcloud;
end

% read time stamps into datetime objects
fileID = fopen('timestamps.txt','r');
string = textscan(fileID, '%s', 'delimiter', '\n');
string = string{1};
for x = 1:length(string)
    dt(x,1) = datetime(string{x}, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
end

% create timetable object - make sure same number of time stamps as point
% clouds
for ii = 1:length(PointCloud)
    TimeStamp(ii,1) = dt(ii,1);
end
full_lidarData = timetable(TimeStamp, PointCloud);

%% Create two agents
numOverlap = 50;
mid = floor(length(PointCloud)/2);
agent1 = timetable(TimeStamp(1:mid + numOverlap,1), PointCloud(1:mid+numOverlap,1));
agent2_forward = timetable(TimeStamp(mid-numOverlap:end,1), PointCloud(mid-numOverlap:end,1));
agent2_backward = timetable(TimeStamp(mid-numOverlap:end,1), flip(PointCloud(mid-numOverlap:end,1)));

% % induce fake perturbations
% pert_r = [cos(10), - sin(10), 0; 
%           sin(10), cos(10), 0;
%           0, 0, 1];
% for ii = 1:height(agent2_forward)
%     pert_locations = agent2_forward.Var1(ii,1).Location;
%     if ii >=1 && ii <= numOverlap
%         for jj = 1:size(pert_locations, 1)
%             new_loc = pert_r * pert_locations(jj,:)';
%             pert_locations(jj,:) = new_loc';
%         end
%     end
%     pert_ptCloud = pointCloud(pert_locations);
%     Var1(ii,1) = pert_ptCloud;
% end
% agent2 = timetable(TimeStamp(mid-numOverlap:end,1), flip(Var1));

%% Use recorded LiDAR Data to Build a Map
skipFrames = 10;
downsamplePercent = 0.1; % Downsample the point clouds prior to registration. Downsampling improves both registration accuracy and algorithm speed.

% Create a map builder object
mapBuilder = helperLidarMapBuilder('DownsamplePercent', downsamplePercent);

% Set random number seed
rng(0);

closeDisplay = false;
numFrames    = height(agent1);

tform_ag1 = rigid3d;
tform_ag2 = rigid3d;
for n = 1 : skipFrames : numFrames - skipFrames

    % Get the nth point cloud
    ptCloud_ag1 = agent1.Var1(n);
    ptCloud_ag2 = agent2_backward.Var1(n);

    % Use transformation from previous iteration as initial estimate for
    % current iteration of point cloud registration. (constant velocity)
    initTform_ag1 = tform_ag1;
    initTform_ag2 = tform_ag2;

    % Update map using the point cloud
    tform_ag1 = updateMap(mapBuilder, ptCloud_ag1, initTform_ag1);
    tform_ag2 = updateMap(mapBuilder, ptCloud_ag2, initTform_ag2);

    % Update map display
    updateDisplay(mapBuilder, closeDisplay);
end