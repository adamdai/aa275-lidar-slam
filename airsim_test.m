close all;
clear all;

%% Process AirSim Velodyne Data
addpath('airsim_data');
addpath('airsim_data\Drone1_pcd');

% read pcd data into pointcloud objects
dataDir = 'airsim_data\Drone1_pcd';
files = dir(fullfile(dataDir, '*.pcd'));
for k = 1:length(files)
    filename = files(k).name;
    pcloud = pcread(filename);
	PointCloud(k,1) = pcloud;
end

% % read time stamps into datetime objects
% fileID = fopen('timestamps.txt','r');
% string = textscan(fileID, '%s', 'delimiter', '\n');
% string = string{1};
% for x = 1:length(string)
%     dt(x,1) = datetime(string{x}, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
% end
% 
% create timetable object - make sure same number of time stamps as point
% clouds
% for ii = 1:length(PointCloud)
%     TimeStamp(ii,1) = dt(ii,1);
% end
% full_lidarData = timetable(TimeStamp, PointCloud);
% full_lidarData_backwards = timetable(TimeStamp, flip(PointCloud));

%% Create two agents
% numOverlap = 150;
% mid = floor(length(PointCloud)/2);
% agent1 = timetable(TimeStamp(1:mid+numOverlap,1), PointCloud(1:mid+numOverlap,1));
% agent2_forward = timetable(TimeStamp(mid-numOverlap:end,1), PointCloud(mid-numOverlap:end,1));
% agent2_backward = timetable(TimeStamp(mid-numOverlap:end,1), flip(PointCloud(mid-numOverlap:end,1)));

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

%% LiDAR Odometry
skipFrames = 1;
numFrames = length(PointCloud);
downsamplePercent = 0.1; % Downsample the point clouds prior to registration. Downsampling improves both registration accuracy and algorithm speed.
rng(0); % set random number seed

% Create a map builder object and build map
mapBuilder_full = helperLidarMapBuilder('DownsamplePercent', downsamplePercent);
closeDisplay = false;
tform = rigid3d;
build_map(mapBuilder_full, closeDisplay, PointCloud, skipFrames, tform);

%% LiDAR SLAM
vSet = pcviewset;

% Create a loop closure detector
matchThresh = 0.08;
loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);

% Create a figure for view set display
hFigBefore = figure('Name', 'View Set Display');
hAxBefore = axes(hFigBefore);

% Initialize transformations
absTform   = rigid3d;  % Absolute transformation to reference frame
relTform   = rigid3d;  % Relative transformation between successive scans
initTform  = rigid3d;
maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted

% Parameters
rng(0);                     % random seed for repeatability 
skipFrames = 1;             % frames to skip
downSamplePercent = 0.1;    % downsample for registration
displayRate = 5;           % Update display every 50 frames
regGridSize = 3;
maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted

% Create loop closure detector
matchThresh = 0.08;
loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);

viewId = 1;
for n = 1 : skipFrames : numFrames

    % Read point cloud
    ptCloudOrig = PointCloud(n);

    % Process point cloud
    %   - Segment and remove ground plane
    %   - Segment and remove ego vehicle
    ptCloud = helperProcessPointCloud(ptCloudOrig);

    % Downsample the processed point cloud
    ptCloud = pcdownsample(ptCloud, "random", downsamplePercent);

    firstFrame = (n==1);
    if firstFrame
        % Add first point cloud scan as a view to the view set
        vSet = addView(vSet, viewId, absTform, "PointCloud", ptCloudOrig);

        viewId = viewId + 1;
        ptCloudPrev = ptCloud;
        continue;
    end

%     % Use INS to estimate an initial transformation for registration
%     initTform = helperComputeInitialEstimateFromINS(relTform, ...
%         insDataTable(n-skipFrames:n, :));

    % Compute rigid transformation that registers current point cloud with
    % previous point cloud
    relTform = pcregisterndt(ptCloud, ptCloudPrev, regGridSize, ...
        "InitialTransform", initTform);

    % Update absolute transformation to reference frame (first point cloud)
    absTform = rigid3d( relTform.T * absTform.T );

    % Add current point cloud scan as a view to the view set
    vSet = addView(vSet, viewId, absTform, "PointCloud", ptCloudOrig);

    % Add a connection from the previous view to the current view representing
    % the relative transformation between them
    vSet = addConnection(vSet, viewId-1, viewId, relTform);

    % Detect loop closure candidates
    [loopFound, loopViewId] = detectLoop(loopDetector, ptCloudOrig);

    % A loop candidate was found
    if loopFound
        loopViewId = loopViewId(1);

        % Retrieve point cloud from view set
        ptCloudOrig = vSet.Views.PointCloud( find(vSet.Views.ViewId == loopViewId, 1) );

        % Process point cloud
        ptCloudOld = helperProcessPointCloud(ptCloudOrig);

        % Downsample point cloud
        ptCloudOld = pcdownsample(ptCloudOld, "random", downsamplePercent);

        % Use registration to estimate the relative pose
        [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloudOld, ...
            regGridSize, "MaxIterations", 50);

        acceptLoopClosure = rmse <= maxTolerableRMSE;
        if acceptLoopClosure
            % For simplicity, use a constant, small information matrix for
            % loop closure edges
            infoMat = 0.01 * eye(6);

            % Add a connection corresponding to a loop closure
            vSet = addConnection(vSet, loopViewId, viewId, relTform, infoMat);
        end
    end

    viewId = viewId + 1;

    ptCloudPrev = ptCloud;
    initTform   = relTform;

    if n>1 && mod(n, displayRate) == 1
        hG = plot(vSet, "Parent", hAxBefore);
        drawnow update
    end
end


G = createPoseGraph(vSet);

% Find and highlight loop closure connections
loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)

optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
vSetOptim = updateView(vSet, optimG.Nodes);
plot(vSetOptim, 'Parent', hAxBefore)










