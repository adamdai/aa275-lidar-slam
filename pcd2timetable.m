function [timeStamps, ptClouds] = pcd2timetable(KittiDataDirectoryPath)

    % read pcd data into pointcloud objects
    dataDir = KittiDataDirectoryPath;
    files = dir(fullfile(dataDir, '*.pcd'));
    for k = 1:length(files)
        filename = files(k).name;
        pcloud = pcread(filename);
        ptClouds(k,1) = pcloud;
    end

    % read time stamps into datetime objects
    fileID = fopen('timestamps.txt','r');
    string = textscan(fileID, '%s', 'delimiter', '\n');
    string = string{1};
    for x = 1:length(string)
        dt(x,1) = datetime(string{x}, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
    end

    % make sure same number of time stamps as point clouds
    for ii = 1:length(ptClouds)
        timeStamps(ii,1) = dt(ii,1);
    end
    
    
end