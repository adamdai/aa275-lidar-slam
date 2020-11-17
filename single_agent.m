clear all;
close all; 

%% Scenario 1 
addpath('velodyne_example_data\scenario1');
scenario1_dataDir = 'velodyne_example_data\scenario1\';
files = dir(fullfile(scenario1_dataDir, '*.png'));
for k = 1:length(files)
    filename = files(k).name;
    ptcloud = helperReadPointCloudFromFile(filename);
    PointClouds(k) = ptcloud;
end

% run sim
rng(0);                     % random seed for repeatability 
skipFrames = 5;             % frames to skip
downSamplePercent = 0.1;    % downsample for registration
displayRate = 100;           % Update display every 50 frames
regGridSize = 3;
maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted

% Create a figure for view set display
hFigBefore1 = figure('Name', 'View Set Display');
hAxBefore1 = axes(hFigBefore1);
hFigBefore2 = figure('Name', 'View Set Display');
hAxBefore2 = axes(hFigBefore2);

% create a pointcloud manager for each agent and central computer 
vSet_cent = pcviewset;

% Create loop closure detector
matchThresh = 0.08;
loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);

% Initialize transformations
absTform = rigid3d;
relTform = rigid3d;
initTform = rigid3d;

% init with first scan
viewId = 1;

ptCloud_orig = PointClouds(1);
ptCloud = helperProcessPointCloud(ptCloud_orig);
ptCloud = pcdownsample(ptCloud, "random", downSamplePercent);

vSet_cent = addView(vSet_cent, viewId, absTform, "PointCloud", ptCloud_orig);
ptCloud_prev = ptCloud;


hG1 = plot(vSet_cent, "Parent", hAxBefore1);
hG2 = plot(vSet_cent, "Parent", hAxBefore2);
drawnow update

% TODO: should technically check for loop closure between first two scans

numFrames = length(PointClouds);
totalLoopDetected = 0;
midScan = 1200;
for n = 2: 1 : numFrames
    viewId = viewId + 1;

    % get agent point clouds
    ptCloud_orig = PointClouds(n);

    % Process point cloud and downsample
    %   - Segment and remove ground plane
    %   - Segment and remove ego vehicle
    ptCloud = helperProcessPointCloud(ptCloud_orig);
    ptCloud = pcdownsample(ptCloud, "random", downSamplePercent);

    % Get rigid transformation that registers points clouds - NDT
    relTform = pcregisterndt(ptCloud, ptCloud_prev, regGridSize,...
        "InitialTransform", initTform);
    absTform = rigid3d(relTform.T * absTform.T);
    
    if n == midScan - 300
        absInit = absTform;
    end
        

    % update central viewset
    vSet_cent = addView(vSet_cent, viewId, absTform, "PointCloud", ptCloud_orig);
    vSet_cent = addConnection(vSet_cent, viewId-1, viewId, relTform);

    % loop detection
    [loopFound, loopViewId] = detectLoop(loopDetector, ptCloud_orig);
    if loopFound
        loopViewId = loopViewId(1);
        matchId = vSet_cent.Views.ViewId(loopViewId);
        ptCloud_match = vSet_cent.Views.PointCloud(loopViewId);
%         ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId1, 1));
        ptCloud_mp = helperProcessPointCloud(ptCloud_match);
        ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);

        % register with matching point cloud
        [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloud_mp, ...
            regGridSize, "MaxIterations", 50);

        acceptLoopClosure = rmse <= maxTolerableRMSE;
        if acceptLoopClosure
            infoMat = 0.01*eye(6);
%             vSet_cent = addConnection(vSet_cent, loopViewId1, viewId_ag1, relTform, infoMat);
            vSet_cent = addConnection(vSet_cent, matchId, viewId, relTform, infoMat);
            totalLoopDetected = totalLoopDetected + 1;
        end
    end

    % TODO: central agent sends optimized trajectories back to agents

    % update view id
    ptCloud_prev = ptCloud;
    initTform = relTform;

    % viewset display update
%     if n>1 && mod(n, displayRate) == 2
%         if n <= midScan + 200
%             hG1 = plot(vSet_cent, "Parent", hAxBefore1);
%             drawnow update
%         elseif n >= midScan - 200
%             hG2 = plot(vSet_cent, "Parent", hAxBefore2);
%             drawnow update
%         end
%     end
        
end


% G = createPoseGraph(vSet_cent);
% 
% % Find and highlight loop closure connections
% % loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
% % highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)
% 
% optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
% vSetOptim = updateView(vSet_cent, optimG.Nodes);
% plot(vSetOptim, 'Parent', hAxBefore)
