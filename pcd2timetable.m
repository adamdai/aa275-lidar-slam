function ptClouds = pcd2timetable(KittiDataDirectoryPath)

    % read pcd data into pointcloud objects
    dataDir = KittiDataDirectoryPath;
    files = dir(fullfile(dataDir, '*.pcd'));
    for k = 1:length(files)
        filename = files(k).name;
        pcloud = pcread(filename);
        ptClouds(k,1) = pcloud;
    end
    
    
end