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

% get point clouds from 
pcd_files = dir('velodyne_points\data_pcd\*.pcd');

for k = 1:length(pcd_files)
    filename = pcd_files(k).name;
    pcloud = pcread(filename);
    
end