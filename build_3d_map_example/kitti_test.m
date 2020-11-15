close all;
clear all;
%% 3D LiDAR tutorial
% baseDownloadURL = 'https://github.com/mathworks/udacity-self-driving-data-subset/raw/master/drive_segment_11_18_16/';
% dataFolder      = fullfile(tempdir, 'drive_segment_11_18_16', filesep);
% options         = weboptions('Timeout', Inf);
% 
% lidarFileName = dataFolder + "lidarPointClouds.mat";
% imuFileName   = dataFolder + "imuOrientations.mat";
% gpsFileName   = dataFolder + "gpsSequence.mat";
% 
% folderExists  = exist(dataFolder, 'dir');
% matfilesExist = exist(lidarFileName, 'file') && exist(imuFileName, 'file') ...
%     && exist(gpsFileName, 'file');
% 
% if ~folderExists
%     mkdir(dataFolder);
% end
% 
% if ~matfilesExist
%     disp('Downloading lidarPointClouds.mat (613 MB)...')
%     websave(lidarFileName, baseDownloadURL + "lidarPointClouds.mat", options);
% 
%     disp('Downloading imuOrientations.mat (1.2 MB)...')
%     websave(imuFileName, baseDownloadURL + "imuOrientations.mat", options);
% 
%     disp('Downloading gpsSequence.mat (3 KB)...')
%     websave(gpsFileName, baseDownloadURL + "gpsSequence.mat", options);
% end
% 
% % Load lidar data from MAT-file
% data = load(lidarFileName);
% lidarPointClouds = data.lidarPointClouds;

%% Kitti Velodyne
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
lidarData = timetable(TimeStamp, PointCloud);

%% Use recorded LiDAR Data to Build a Map
frameNum = 100;
ptCloud_id = lidarData.PointCloud(frameNum);
% helperVisualizeEgoView(ptCloud_id);

skipFrames = 10;
fixed  = lidarData.PointCloud(frameNum);
moving = lidarData.PointCloud(frameNum + skipFrames);

fixedProcessed  = helperProcessPointCloud(fixed);
movingProcessed = helperProcessPointCloud(moving);

% visualize processed point cloud
% hFigFixed = figure;
% pcshowpair(fixed, fixedProcessed)
% view(2);                            % Adjust view to show top-view

% Downsample the point clouds prior to registration. Downsampling improves
% both registration accuracy and algorithm speed.
downsamplePercent = 0.1;
fixedDownsampled  = pcdownsample(fixedProcessed, 'random', downsamplePercent);
movingDownsampled = pcdownsample(movingProcessed, 'random', downsamplePercent);

regGridStep = 5;
tform = pcregisterndt(movingDownsampled, fixedDownsampled, regGridStep);
movingReg = pctransform(movingProcessed, tform);

% Visualize alignment in top-view before and after registration
% hFigAlign = figure;
% 
% subplot(121)
% pcshowpair(movingProcessed, fixedProcessed)
% title('Before Registration')
% view(2)
% 
% subplot(122)
% pcshowpair(movingReg, fixedProcessed)
% title('After Registration')
% view(2)
% 
% helperMakeFigurePublishFriendly(hFigAlign);

% Create a map builder object
mapBuilder = helperLidarMapBuilder('DownsamplePercent', downsamplePercent);

% Set random number seed
rng(0);

closeDisplay = false;
numFrames    = height(lidarData);

tform = rigid3d;
for n = 1 : skipFrames : numFrames - skipFrames

    % Get the nth point cloud
    ptCloud = lidarData.PointCloud(n);

    % Use transformation from previous iteration as initial estimate for
    % current iteration of point cloud registration. (constant velocity)
    initTform = tform;

    % Update map using the point cloud
    tform = updateMap(mapBuilder, ptCloud, initTform);

    % Update map display
    updateDisplay(mapBuilder, closeDisplay);
end