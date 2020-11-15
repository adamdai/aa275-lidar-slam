%% Map and Lidar
% Create example binary occupancy map
p = zeros(100,100);
p(1:20,1:20) = ones(20,20);
p(51:100,31:50) = ones(50,20);
p(11:40,61:80) = ones(30,20);
p(71:80,61:100) = ones(10,40);
p(51:60,11:30) = ones(10,20);
resolution = 1;
map = binaryOccupancyMap(p, resolution);

% Simulate lidar sensor. Use default detection angle of [-pi pi]
lidar = rangeSensor;
% Set min and max values of the detectable range of the sensor in meters
maxRange = 50;
lidar.Range = [0 maxRange];

%% SLAM 

slamObj = lidarSLAM(resolution, maxRange);
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

%% Trajectory
% Arrival, Waypoints, Orientation
constraints = [0,    0,0,0,     0,0,0;
               1,    10,20,0,   0,0,0;
               2,    5,50,0,    0,0,0;
               3,    40,70,0    0,0,0;
               4,    60,50,0    0,0,0;
               5,    80,40,0    0,0,0;
               6,    90,80,0    0,0,0;
               7,    100,100,0    0,0,0];

trajectory = waypointTrajectory(constraints(:,2:4), ...
    'TimeOfArrival',constraints(:,1));

tInfo = waypointInfo(trajectory);

figure(1)
hold on
show(map)
plot(tInfo.Waypoints(1,1),tInfo.Waypoints(1,2),'b*')
grid on

orient = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,'quaternion');
vel = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);
acc = vel;
angVel = vel;

count = 1;
while ~isDone(trajectory)
    % update trajectory
    [pos,orient(count),vel(count,:),acc(count,:),angVel(count,:)] = trajectory();
    % plot new robot position
    plot(pos(1),pos(2),'b.')
    
    % get new scan from lidar
    [ranges,angles] = lidar(pos,map);
    local_scan = lidarScan(ranges,angles);
   
    % shift scan to global frame and plot it
    global_scan = lidarScan(local_scan.Cartesian + [pos(1),pos(2)]);
    plot(global_scan)
    xlim([0 100])
    ylim([0 100])

    % add scan to SLAM object
    addScan(slamObj, local_scan);
%     if rem(count,10) == 0
%         show(slamObj);
%     end

    pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
    count = count + 1;
end

show(slamObj)