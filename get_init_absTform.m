function [absTform, relTform] = get_init_absTform(PointClouds, stop_frame)
% run sim
rng(0);                     % random seed for repeatability 
downSamplePercent = 0.1;    % downsample for registration
regGridSize = 3;
maxTolerableRMSE  = 3; % Maximum allowed RMSE for a loop closure candidate to be accepted

% Create loop closure detector
matchThresh = 0.08;
loopDetector = helperLoopClosureDetector('MatchThreshold', matchThresh);

% Initialize transformations
absTform = rigid3d;
relTform = rigid3d;
initTform = rigid3d;

% init with first scan
viewId = 1;
vSet_cent = pcviewset;
ptCloud_orig = PointClouds(1);
ptCloud = helperProcessPointCloud(ptCloud_orig);
ptCloud = pcdownsample(ptCloud, "random", downSamplePercent);
vSet_cent = addView(vSet_cent, viewId, absTform, "PointCloud", ptCloud_orig);
ptCloud_prev = ptCloud;

for n = 2: 1 : stop_frame
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
        
    % update central viewset
    vSet_cent = addView(vSet_cent, viewId, absTform, "PointCloud", ptCloud_orig);
    vSet_cent = addConnection(vSet_cent, viewId-1, viewId, relTform);

%     % loop detection
%     [loopFound, loopViewId] = detectLoop(loopDetector, ptCloud_orig);
%     if loopFound
%         loopViewId = loopViewId(1);
%         matchId = vSet_cent.Views.ViewId(loopViewId);
%         ptCloud_match = vSet_cent.Views.PointCloud(loopViewId);
%         ptCloud_mp = helperProcessPointCloud(ptCloud_match);
%         ptCloud_mp = pcdownsample(ptCloud_mp, "random", downSamplePercent);
% 
%         % register with matching point cloud
%         [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloud_mp, ...
%             regGridSize, "MaxIterations", 50);
% 
%         acceptLoopClosure = rmse <= maxTolerableRMSE;
%         if acceptLoopClosure
%             infoMat = 0.01*eye(6);
%             vSet_cent = addConnection(vSet_cent, matchId, viewId, relTform, infoMat);
%         end
%     end
    
    % update view id
    ptCloud_prev = ptCloud;
    initTform = relTform;
end

end