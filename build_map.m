function build_map(mapBuilder, closeDisplay, ptCloudData, skipFrames, tform)
numFrames    = length(ptCloudData);

for n = 1 : skipFrames : numFrames - skipFrames
    % Get the nth point cloud
    ptCloud = ptCloudData(n);

    % Use transformation from previous iteration as initial estimate for
    % current iteration of point cloud registration. (constant velocity)
    initTform = tform;

    % Update map using the point cloud
    tform = updateMap(mapBuilder, ptCloud, initTform);

    % Update map display
    updateDisplay(mapBuilder, closeDisplay);
end

end