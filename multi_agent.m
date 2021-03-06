close all;
clear all;
warning off;

%% Process Kitti Velodyne Data
% % kitti velodyne data
% addpath('velodyne_points');
% addpath('velodyne_points\data_pcd');
% addpath('helpers');
% [TimeStamp, PointCloud] = pcd2timetable('velodyne_points\data_pcd\');
% 
% % create two agents
% numOverlap = 100;
% midScan = floor(length(PointCloud)/2);
% agent1 = timetable(TimeStamp(1:midScan+numOverlap,1), PointCloud(1:midScan+numOverlap,1));
% agent2_forward = timetable(TimeStamp(midScan-numOverlap:end,1), PointCloud(midScan-numOverlap:end,1));
% agent2_backward = timetable(TimeStamp(midScan-numOverlap:end,1), flip(PointCloud(midScan-numOverlap:end,1)));
% 
% % % induce noise
% % pert_r = [cos(10), - sin(10), 0; 
% %           sin(10), cos(10), 0;
% %           0, 0, 1];
% % for ii = 1:height(agent2_forward)
% %     pert_locations = agent2_forward.Var1(ii,1).Location;
% %     if ii >=1 && ii <= numOverlap
% %         for jj = 1:size(pert_locations, 1)
% %             new_loc = pert_r * pert_locations(jj,:)';
% %             pert_locations(jj,:) = new_loc';
% %         end
% %     end
% %     pert_ptCloud = pointCloud(pert_locations);
% %     Var1(ii,1) = pert_ptCloud;
% % end
% % agent2 = timetable(TimeStamp(mid-numOverlap:end,1), flip(Var1));
% 
% rng(0);                     % random seed for repeatability 
% skipFrames = 5;             % frames to skip
% downSamplePercent = 0.1;    % downsample for registration
% displayRate = 50;           % Update display every 50 frames
% regGridSize = 3;
% maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted
% 
% % Create a figure for view set display
% hFigBefore = figure('Name', 'View Set Display');
% hAxBefore = axes(hFigBefore);
% 
% % create a pointcloud manager for each agent and central computer 
% vSet_ag1 = pcviewset;
% vSet_ag2 = pcviewset;
% vSet_cent = pcviewset;
% % map_builder_cent = helperLidarMapBuilder('DownsamplePercent', downSamplePercent);
% 
% % Create loop closure detector
% matchThresh = 0.08;
% loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);
% 
% % Initialize transformations
% absTform_ag1 = rigid3d;
% absTform_ag2 = rigid3d;
% relTform_ag1 = rigid3d;
% relTform_ag2 = rigid3d;
% initTform_ag1 = rigid3d;
% initTform_ag2 = rigid3d;
% 
% % init with first scan
% viewId = 1;
% viewId_ag1 = viewId;
% viewId_ag2 = viewId + height(agent1);
% ptCloud1_orig = agent1.Var1(1);
% ptCloud2_orig = agent2_forward.Var1(1);
% ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
% ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
% ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
% ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
% 
% vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% ptCloud1_prev = ptCloud1;
% ptCloud2_prev = ptCloud2;
% hG = plot(vSet_cent, "Parent", hAxBefore);
% drawnow update
% 
% % TODO: should technically check for loop closure between first two scans
% 
% % numFrames = min(height(agent1), height(agent2_backward)); % should be max, uneven data sets
% numFrames = length(PointClouds);
% for n = 2: skipFrames : numFrames
%     viewId = viewId + 1;
%     viewId_ag1 = viewId;
%     viewId_ag2 = viewId + height(agent1);
%     
% %     try
%         % get agent point clouds
%         ptCloud1_orig = agent1.Var1(n);
%         ptCloud2_orig = agent2_forward.Var1(n);
%         ptCloud1_orig = PointClouds(n);
% 
%         % Process point cloud and downsample
%         %   - Segment and remove ground plane
%         %   - Segment and remove ego vehicle
%         ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
%         ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
%         ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
%         ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
%         
%         % Get rigid transformation that registers points clouds - NDT
%         relTform_ag1 = pcregisterndt(ptCloud1, ptCloud1_prev, regGridSize,...
%             "InitialTransform", initTform_ag1);
%         relTform_ag2 = pcregisterndt(ptCloud2, ptCloud2_prev, regGridSize,...
%             "InitialTransform", initTform_ag2);
%         absTform_ag1 = rigid3d(relTform_ag1.T * absTform_ag1.T);
%         absTform_ag2 = rigid3d(relTform_ag2.T * absTform_ag2.T);
%         
%         % agents add new view and connect to previous view
%         vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%         vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%         vSet_ag1 = addConnection(vSet_ag1, viewId_ag1-1, viewId_ag1, relTform_ag1);
%         vSet_ag2 = addConnection(vSet_ag2, viewId_ag2-1, viewId_ag2, relTform_ag2);
%         
%         % update central viewset
%         vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%         vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%         vSet_cent = addConnection(vSet_cent, viewId_ag1-1, viewId_ag1, relTform_ag1);
%         vSet_cent = addConnection(vSet_cent, viewId_ag2-1, viewId_ag2, relTform_ag2);
%         
%         % loop detection
%         [loopFound1, loopViewId1] = detectLoop(loopDetector, ptCloud1_orig);
%         if loopFound1
%             loopViewId1 = loopViewId1(1);
%             ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId1, 1));
%             ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%             ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
%             
%             % register with matching point cloud
%             [relTform, ~, rmse] = pcregisterndt(ptCloud1, ptCloud_mp, ...
%                 regGridSize, "MaxIterations", 50);
%             
%             acceptLoopClosure = rmse <= maxTolerableRMSE;
%             if acceptLoopClosure
%                 infoMat = 0.01*eye(6);
%                 vSet_cent = addConnection(vSet_cent, loopViewId1, viewId_ag1, relTform, infoMat);
%             end
%         end
% 
%         [loopFound2, loopViewId2] = detectLoop(loopDetector, ptCloud2_orig);
%         if loopFound2
%             loopViewId2 = loopViewId2(1);
%             ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId2, 1));
%             ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%             ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
%             
%             % register with matching point cloud
%             [relTform, ~, rmse] = pcregisterndt(ptCloud2, ptCloud_mp, ...
%                 regGridSize, "MaxIterations", 50);
%             
%             acceptLoopClosure = rmse <= maxTolerableRMSE;
%             if acceptLoopClosure
%                 infoMat = 0.01*eye(6);
%                 vSet_cent = addConnection(vSet_cent, loopViewId2, viewId_ag2, relTform, infoMat);
%             end
%         end
%         
%         % TODO: central agent sends optimized trajectories back to agents
%         
%         % update view id
%         ptCloud1_prev = ptCloud1;
%         ptCloud2_prev = ptCloud2;
%         initTform_ag1 = relTform_ag1;
%         initTform_ag2 = relTform_ag2;
%         
%         G = createPoseGraph(vSet_cent);
%         optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
%         vSet_cent = updateView(vSet_cent, optimG.Nodes);
%         
%         % viewset display update
%         if n>1 && mod(n, displayRate) == 2
%             hG = plot(vSet_cent, "Parent", hAxBefore);
%             drawnow update
%         end
%         
% 
% %         plot(vSetOptim, 'Parent', hAxBefore)
%         
% %         % Update map display
% %         temp = updateMap(map_builder_cent, ptCloud1_orig, relTform_ag1);
% %         updateDisplay(map_builder_cent, false);
%         
% %     catch
% %     end 
% end
% 
% 
% % G = createPoseGraph(vSet_cent);
% % 
% % % Find and highlight loop closure connections
% % loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
% % highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)
% % 
% % optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
% % vSetOptim = updateView(vSet_cent, optimG.Nodes);
% % plot(vSetOptim, 'Parent', hAxBefore)
% 
% 

% %% Scenario 1 
% addpath('velodyne_example_data\scenario1');
% addpath('helpers');
% scenario1_dataDir = 'velodyne_example_data\scenario1\';
% files = dir(fullfile(scenario1_dataDir, '*.png'));
%  for k = 1:length(files)
%     filename = files(k).name;
%     ptcloud = helperReadPointCloudFromFile(filename);
%     PointClouds(k) = ptcloud;
%  end
%  
% % create two agents
% numOverlap = 200;
% midScan = 1200;
% agent1_ptc = PointClouds(1:midScan + numOverlap);
% agent2_ptc = PointClouds(midScan - numOverlap:end);
% 
% % run sim
% rng(0);                     % random seed for repeatability 
% skipFrames = 5;             % frames to skip
% downSamplePercent = 0.1;    % downsample for registration
% displayRate = 100;          % Update display every 50 frames
% regGridSize = 3;
% maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted
% 
% % Create a figure for view set display
% hFigBefore = figure('Name', 'View Set Display');
% hAxBefore = axes(hFigBefore);
% 
% % create a pointcloud manager for each agent and central computer 
% vSet_ag1 = pcviewset;
% vSet_ag2 = pcviewset;
% vSet_cent = pcviewset;
% 
% % Create loop closure detector
% matchThresh = 0.08;
% loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);
% 
% % Initialize transformations
% absTform_ag1 = rigid3d;
% relTform_ag1 = rigid3d;
% initTform_ag1 = rigid3d;
% 
% [absTform_ag2, initTform_ag2] = get_init_absTform(PointClouds, midScan-numOverlap);
% % absTform_ag2 = rigid3d;
% relTform_ag2 = rigid3d;
% % initTform_ag2 = rigid3d;
% 
% % init with first scan
% viewId = 1;
% viewId_ag1 = viewId;
% viewId_ag2 = viewId + length(agent1_ptc);
% 
% ptCloud1_orig = agent1_ptc(1);
% ptCloud2_orig = agent2_ptc(1);
% 
% ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
% ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
% 
% ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
% ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
% 
% vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% ptCloud1_prev = ptCloud1;
% ptCloud2_prev = ptCloud2;
% hG = plot(vSet_cent, "Parent", hAxBefore);
% frame = getframe(hFigBefore);
% im = frame2im(frame);
% [imind, cm] = rgb2ind(im, 256);
% imwrite(imind, cm, 'scenario1_ma.gif', 'gif', 'Loopcount', inf);
% drawnow update
% 
% % TODO: should technically check for loop closure between first two scans
% 
% numFrames = min(length(agent1_ptc), length(agent2_ptc));
% totalLoopDetected = 0;
% for n = 2: skipFrames : numFrames
%     viewId = viewId + 1;
%     viewId_ag1 = viewId;
%     viewId_ag2 = viewId + length(agent1_ptc);
% 
%     % get agent point clouds
%     ptCloud1_orig = agent1_ptc(n);
%     ptCloud2_orig = agent2_ptc(n);
% 
%     % Process point cloud and downsample
%     %   - Segment and remove ground plane
%     %   - Segment and remove ego vehicle
%     ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
%     ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
%     
%     ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
%     ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
% 
%     % Get rigid transformation that registers points clouds - NDT
%     relTform_ag1 = pcregisterndt(ptCloud1, ptCloud1_prev, regGridSize,...
%         "InitialTransform", initTform_ag1);
%     relTform_ag2 = pcregisterndt(ptCloud2, ptCloud2_prev, regGridSize,...
%         "InitialTransform", initTform_ag2);
%     absTform_ag1 = rigid3d(relTform_ag1.T * absTform_ag1.T);
%     absTform_ag2 = rigid3d(relTform_ag2.T * absTform_ag2.T);
% 
%     % agents add new view and connect to previous view
%     vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%     vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%     vSet_ag1 = addConnection(vSet_ag1, viewId_ag1-1, viewId_ag1, relTform_ag1);
%     vSet_ag2 = addConnection(vSet_ag2, viewId_ag2-1, viewId_ag2, relTform_ag2);
% 
%     % update central viewset
%     vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%     vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%     vSet_cent = addConnection(vSet_cent, viewId_ag1-1, viewId_ag1, relTform_ag1);
%     vSet_cent = addConnection(vSet_cent, viewId_ag2-1, viewId_ag2, relTform_ag2);
% 
%     % loop detection
%     [loopFound1, loopViewId1] = detectLoop(loopDetector, ptCloud1_orig);
%     [loopFound2, loopViewId2] = detectLoop(loopDetector, ptCloud2_orig);
%     if loopFound1
%         loopViewId1 = loopViewId1(1);
%         matchId = vSet_cent.Views.ViewId(loopViewId1);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId1);
% %         ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId1, 1));
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud1, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             infoMat = 0.01*eye(6);
% %             vSet_cent = addConnection(vSet_cent, loopViewId1, viewId_ag1, relTform, infoMat);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId_ag1, relTform, infoMat);
%             totalLoopDetected = totalLoopDetected + 1;
%         end
%     end
%     if loopFound2
%         loopViewId2 = loopViewId2(1);
% %         ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId2, 1));
%         matchId = vSet_cent.Views.ViewId(loopViewId2);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId2);
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud2, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             infoMat = 0.01*eye(6);
% %             vSet_cent = addConnection(vSet_cent, loopViewId2, viewId_ag2, relTform, infoMat);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId_ag2, relTform, infoMat);
%             totalLoopDetected = totalLoopDetected + 1;
%         end
%     end
% 
%     % TODO: central agent sends optimized trajectories back to agents
% 
%     % update view id
%     ptCloud1_prev = ptCloud1;
%     ptCloud2_prev = ptCloud2;
%     initTform_ag1 = relTform_ag1;
%     initTform_ag2 = relTform_ag2;
% 
% %     % optimize posegraph
% %     G = createPoseGraph(vSet_cent);
% %     optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
% %     
% % %     % Find and highlight loop closure connections
% % %     try
% % %         loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
% % %         highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)
% % %     catch
% % %         print("no loop closures found");
% % %     end
% %     
% %     vSet_cent = updateView(vSet_cent, optimG.Nodes);
% 
%     % viewset display update
%     if n>1 && mod(n, displayRate) == 2
%         hG = plot(vSet_cent, "Parent", hAxBefore);
%         frame = getframe(hFigBefore);
%         im = frame2im(frame);
%         [imind, cm] = rgb2ind(im, 256);
%         imwrite(imind, cm, 'scenario1_ma.gif', 'gif', 'WriteMode', 'append');
%         drawnow update
%     end
%         
% end
% 
% G = createPoseGraph(vSet_cent);
% 
% % Find and highlight loop closure connections
% % loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
% % highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)
% 
% optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
% vSetOptim = updateView(vSet_cent, optimG.Nodes);
% plot(vSetOptim, 'Parent', hAxBefore)


%% Scenario 1 
addpath('velodyne_example_data\scenario1');
addpath('helpers');
scenario1_dataDir = 'velodyne_example_data\scenario1\';
files = dir(fullfile(scenario1_dataDir, '*.png'));
 for k = 1:length(files)
    filename = files(k).name;
    ptcloud = helperReadPointCloudFromFile(filename);
    PointClouds(k) = ptcloud;
 end
 
% create two agents
numOverlap = 250;
midScan = 1200;
agent1_ptc = PointClouds(1:midScan + numOverlap);
agent2_ptc = PointClouds(midScan - numOverlap:end);

% run sim
rng(0);                     % random seed for repeatability 
% skipFrames = 5;             % frames to skip
% downSamplePercent = 0.1;    % downsample for registration
% displayRate = 100;          % Update display every 50 frames
% regGridSize = 3;
% maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted
% 
% % Create a figure for view set display
% hFigBefore = figure('Name', 'View Set Display');
% hAxBefore = axes(hFigBefore);
% 
% % create a pointcloud manager for each agent and central computer 
% vSet_ag1 = pcviewset;
% vSet_ag2 = pcviewset;
% vSet_cent = pcviewset;
% 
% % Create loop closure detector
% matchThresh = 0.08;
% loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);
% 
% % Initialize transformations
% absTform_ag1 = rigid3d;
% relTform_ag1 = rigid3d;
% initTform_ag1 = rigid3d;
% 
% [absTform_ag2, initTform_ag2] = get_init_absTform(PointClouds, midScan-numOverlap);
% % absTform_ag2 = rigid3d;
% relTform_ag2 = rigid3d;
% % initTform_ag2 = rigid3d;
% 
% % init with first scan
% viewId = 1;
% viewId_ag1 = viewId;
% viewId_ag2 = viewId + length(agent1_ptc);
% 
% ptCloud1_orig = agent1_ptc(1);
% ptCloud2_orig = agent2_ptc(1);
% 
% ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
% ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
% 
% ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
% ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
% 
% vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% ptCloud1_prev = ptCloud1;
% ptCloud2_prev = ptCloud2;
% hG = plot(vSet_cent, "Parent", hAxBefore);
% frame = getframe(hFigBefore);
% im = frame2im(frame);
% [imind, cm] = rgb2ind(im, 256);
% imwrite(imind, cm, 'scenario1_ma.gif', 'gif', 'Loopcount', inf);
% drawnow update
% 
% % TODO: should technically check for loop closure between first two scans
% 
% numFrames = min(length(agent1_ptc), length(agent2_ptc));
% for n = 2: skipFrames : numFrames
%     viewId = viewId + 1;
%     viewId_ag1 = viewId;
%     viewId_ag2 = viewId + length(agent1_ptc);
% 
%     % get agent point clouds
%     ptCloud1_orig = agent1_ptc(n);
%     ptCloud2_orig = agent2_ptc(n);
% 
%     % Process point cloud and downsample
%     %   - Segment and remove ground plane
%     %   - Segment and remove ego vehicle
%     ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
%     ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
%     
%     ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
%     ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
% 
%     % Get rigid transformation that registers points clouds - NDT
%     relTform_ag1 = pcregisterndt(ptCloud1, ptCloud1_prev, regGridSize,...
%         "InitialTransform", initTform_ag1);
%     relTform_ag2 = pcregisterndt(ptCloud2, ptCloud2_prev, regGridSize,...
%         "InitialTransform", initTform_ag2);
%     absTform_ag1 = rigid3d(relTform_ag1.T * absTform_ag1.T);
%     absTform_ag2 = rigid3d(relTform_ag2.T * absTform_ag2.T);
% 
%     % agents add new view and connect to previous view
%     vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%     vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%     vSet_ag1 = addConnection(vSet_ag1, viewId_ag1-1, viewId_ag1, relTform_ag1);
%     vSet_ag2 = addConnection(vSet_ag2, viewId_ag2-1, viewId_ag2, relTform_ag2);
% 
%     % update central viewset
%     vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%     vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%     vSet_cent = addConnection(vSet_cent, viewId_ag1-1, viewId_ag1, relTform_ag1);
%     vSet_cent = addConnection(vSet_cent, viewId_ag2-1, viewId_ag2, relTform_ag2);
% 
%     % loop detection
%     [loopFound1, loopViewId1] = detectLoop(loopDetector, ptCloud1_orig);
%     [loopFound2, loopViewId2] = detectLoop(loopDetector, ptCloud2_orig);
%     if loopFound1
%         loopViewId1 = loopViewId1(1);
%         matchId = vSet_cent.Views.ViewId(loopViewId1);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId1);
% %         ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId1, 1));
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud1, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             infoMat = 0.01*eye(6);
% %             vSet_cent = addConnection(vSet_cent, loopViewId1, viewId_ag1, relTform, infoMat);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId_ag1, relTform, infoMat);
%         end
%     end
%     if loopFound2
%         loopViewId2 = loopViewId2(1);
% %         ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId2, 1));
%         matchId = vSet_cent.Views.ViewId(loopViewId2);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId2);
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud2, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             infoMat = 0.01*eye(6);
% %             vSet_cent = addConnection(vSet_cent, loopViewId2, viewId_ag2, relTform, infoMat);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId_ag2, relTform, infoMat);
%         end
%     end
% 
%     % TODO: central agent sends optimized trajectories back to agents
% 
%     % update view id
%     ptCloud1_prev = ptCloud1;
%     ptCloud2_prev = ptCloud2;
%     initTform_ag1 = relTform_ag1;
%     initTform_ag2 = relTform_ag2;
% 
%     % optimize posegraph
%     G = createPoseGraph(vSet_cent);
%     optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
%     
% %     % Find and highlight loop closure connections
% %     try
% %         loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
% %         highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)
% %     catch
% %         print("no loop closures found");
% %     end
%     
%     vSet_cent = updateView(vSet_cent, optimG.Nodes);
% 
%     % viewset display update
%     if n>1 && mod(n, displayRate) == 2
%         hG = plot(vSet_cent, "Parent", hAxBefore);
%         frame = getframe(hFigBefore);
%         im = frame2im(frame);
%         [imind, cm] = rgb2ind(im, 256);
%         imwrite(imind, cm, 'scenario1_ma.gif', 'gif', 'WriteMode', 'append');
%         drawnow update
%     end
%         
% end
% 
% % G = createPoseGraph(vSet_cent);
% % 
% % % Find and highlight loop closure connections
% % % loopEdgeIds = find(abs(diff(G.Edges.EndNodes, 1, 2)) > 1);
% % % highlight(hG, 'Edges', loopEdgeIds, 'EdgeColor', 'red', 'LineWidth', 3)
% % 
% % optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
% % vSetOptim = updateView(vSet_cent, optimG.Nodes);
% % plot(vSetOptim, 'Parent', hAxBefore)

% map builderN
tform1 = rigid3d;
% tform2 = rigid3d;
skipFrames = 5;             % frames to skip
downSamplePercent = 0.1;    % downsample for registration
mapBuilder = helperLidarMapBuilder('DownsamplePercent', downSamplePercent);
numFrames1 = length(agent1_ptc);
numFrames2 = length(agent2_ptc);
[absTform_ag2, tform2] = get_init_absTform(PointClouds, midScan-numOverlap);
closeDisplay = false;
for n = 1 : skipFrames : numFrames1
    % Get the nth point cloud
    ptCloud1 = agent1_ptc(n);

    % Use transformation from previous iteration as initial estimate for
    % current iteration of point cloud registration. (constant velocity)
    initTform1 = tform1;
    

    % Update map using the point cloud
    tform1 = updateMap(mapBuilder, ptCloud1, initTform1);
%     optimizeMapPoses(mapBuilder);

    % Update map display
    updateDisplay(mapBuilder, closeDisplay);
    
end
for n = 1:skipFrames:numFrames2
    ptCloud2 = agent2_ptc(n);
    initTform2 = tform2;
    if n == 1
        tform2 = updateMap(mapBuilder, ptCloud2, initTform2, absTform_ag2);
    else
        tform2 = updateMap(mapBuilder, ptCloud2, initTform2);
    end
    optimizeMapPoses(mapBuilder);
    updateDisplay(mapBuilder, closeDisplay);
end

%% airsim
% % airsim data
% addpath('airsim_data');
% addpath('airsim_data\Drone0_pcd');
% addpath('airsim_data\Drone1_pcd');
% addpath('helpers');
% agent1_ptc = pcd2timetable('airsim_data\Drone0_pcd\');
% agent2_ptc = pcd2timetable('airsim_data\Drone1_pcd\');
% 
% % run sim
% rng(0);                     % random seed for repeatability 
% skipFrames = 1;             % frames to skip
% downSamplePercent = 0.8;    % downsample for registration
% displayRate = 3;            % Update display every __ frames
% regGridSize = 3;
% % regGridSize = 10;
% % maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted
% maxTolerableRMSE  = 5;
% 
% % Create a figure for view set display
% hFigBefore = figure('Name', 'View Set Display');
% hAxBefore = axes(hFigBefore);
% 
% % create a pointcloud manager for each agent and central computer 
% vSet_ag1 = pcviewset;
% vSet_ag2 = pcviewset;
% vSet_cent = pcviewset;
% 
% % Create loop closure detector
% % matchThresh = 0.08;
% % matchThresh = 0.5;
% matchThresh = 0.2;
% loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);
% 
% % Initialize transformations
% absTform_ag1 = rigid3d;
% relTform_ag1 = rigid3d;
% initTform_ag1 = rigid3d;
% 
% absTform_ag2 = rigid3d;
% relTform_ag2 = rigid3d;
% initTform_ag2 = rigid3d;
% 
% % init with first scan
% viewId = 1;
% viewId_ag1 = viewId;
% viewId_ag2 = viewId + length(agent1_ptc);
% 
% ptCloud1_orig = agent1_ptc(1);
% ptCloud2_orig = agent2_ptc(1);
% 
% % ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
% % ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
% ptCloud1 = pcdownsample(ptCloud1_orig, "random", downSamplePercent);
% 
% % ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
% % ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
% ptCloud2 = pcdownsample(ptCloud2_orig, "random", downSamplePercent);
% 
% vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
% vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
% 
% ptCloud1_prev = ptCloud1;
% ptCloud2_prev = ptCloud2;
% hG = plot(vSet_cent, "Parent", hAxBefore);
% frame = getframe(hFigBefore);
% im = frame2im(frame);
% [imind, cm] = rgb2ind(im, 256);
% imwrite(imind, cm, 'air_sim.gif', 'gif', 'Loopcount', inf);
% drawnow update
% 
% % TODO: should technically check for loop closure between first two scans
% 
% numFrames = max(length(agent1_ptc), length(agent2_ptc));
% nLoops = 0;
% for n = 2: skipFrames : numFrames
%     viewId = viewId + 1;
%     viewId_ag1 = viewId;
%     viewId_ag2 = viewId + length(agent1_ptc);
% 
%     % process point cloud
%     ptCloud1_orig = agent1_ptc(n);
% %         ptCloud1 = helperProcessPointCloud(ptCloud1_orig);
% %         ptCloud1 = pcdownsample(ptCloud1, "random", downSamplePercent);
%     ptCloud1 = pcdownsample(ptCloud1_orig, "random", downSamplePercent);
% 
%     % register and get transforms
%     relTform_ag1 = pcregisterndt(ptCloud1, ptCloud1_prev, regGridSize,...
%         "InitialTransform", initTform_ag1);
%     absTform_ag1 = rigid3d(relTform_ag1.T * absTform_ag1.T);
% 
%     % update viewsets
%     vSet_ag1 = addView(vSet_ag1, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%     vSet_ag1 = addConnection(vSet_ag1, viewId_ag1-1, viewId_ag1, relTform_ag1);
%     vSet_cent = addView(vSet_cent, viewId_ag1, absTform_ag1, "PointCloud", ptCloud1_orig);
%     vSet_cent = addConnection(vSet_cent, viewId_ag1-1, viewId_ag1, relTform_ag1);
%     
%     % process point cloud
%     ptCloud2_orig = agent2_ptc(n);
% %         ptCloud2 = helperProcessPointCloud(ptCloud2_orig);
% %         ptCloud2 = pcdownsample(ptCloud2, "random", downSamplePercent);
%     ptCloud2 = pcdownsample(ptCloud2_orig, "random", downSamplePercent);
% 
%     relTform_ag2 = pcregisterndt(ptCloud2, ptCloud2_prev, regGridSize,...
%         "InitialTransform", initTform_ag2);
%     absTform_ag2 = rigid3d(relTform_ag2.T * absTform_ag2.T);
% 
%     % update viewsets
%     vSet_ag2 = addView(vSet_ag2, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%     vSet_ag2 = addConnection(vSet_ag2, viewId_ag2-1, viewId_ag2, relTform_ag2);
%     vSet_cent = addView(vSet_cent, viewId_ag2, absTform_ag2, "PointCloud", ptCloud2_orig);
%     vSet_cent = addConnection(vSet_cent, viewId_ag2-1, viewId_ag2, relTform_ag2);
%    
%     % loop detection
%     [loopFound1, loopViewId1] = detectLoop(loopDetector, ptCloud1_orig);
%     if loopFound1
%         disp('detected');
%         loopViewId1 = loopViewId1(1);
% %             ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId1, 1));
%         matchId = vSet_cent.Views.ViewId(loopViewId1);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId1);
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud1, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             disp('loop closure')
%             nLoops = nLoops + 1;
%             infoMat = 0.01*eye(6);
% %             vSet_cent = addConnection(vSet_cent, loopViewId1, viewId_ag1, relTform, infoMat);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId_ag1, relTform, infoMat);
%         end
%     end
%     
%     [loopFound2, loopViewId2] = detectLoop(loopDetector, ptCloud2_orig);
%     if loopFound2
%         disp('detected');
%         loopViewId2 = loopViewId2(1);
% %         ptCloud_match = vSet_cent.Views.PointCloud(find(vSet_cent.Views.ViewId == loopViewId2, 1));
%         matchId = vSet_cent.Views.ViewId(loopViewId2);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId2);
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud2, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             disp('loop closure')
%             nLoops = nLoops + 1;
%             infoMat = 0.01*eye(6);
% %             vSet_cent = addConnection(vSet_cent, loopViewId2, viewId_ag2, relTform, infoMat);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId_ag2, relTform, infoMat);
%         end
%     end
% 
%     % TODO: central agent sends optimized trajectories back to agents
% 
%     % update initial relative transform
%     ptCloud1_prev = ptCloud1;
%     initTform_ag1 = relTform_ag1;
%     ptCloud2_prev = ptCloud2;
%     initTform_ag2 = relTform_ag2;
% 
%     % viewset display update
%     if n>1 && mod(n, displayRate) == 2
%         hG = plot(vSet_cent, "Parent", hAxBefore);
%         frame = getframe(hFigBefore);
%         im = frame2im(frame);
%         [imind, cm] = rgb2ind(im, 256);
%         imwrite(imind, cm, 'air_sim.gif', 'gif', 'WriteMode', 'append');
%         drawnow update
%     end
%         
% end
% 
% G = createPoseGraph(vSet_cent);
% optimG = optimizePoseGraph(G, 'g2o-levenberg-marquardt');
% vSetOptim = updateView(vSet_cent, optimG.Nodes);
% plot(vSetOptim, 'Parent', hAxBefore)
% frame = getframe(hFigBefore);
% im = frame2im(frame);
% [imind, cm] = rgb2ind(im, 256);
% imwrite(imind, cm, 'air_sim.gif', 'gif', 'WriteMode', 'append');
