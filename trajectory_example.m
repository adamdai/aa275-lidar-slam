%% Map and Lidar
% Create example binary occupancy map
p = zeros(100,100);
p(1:20,1:20) = ones(20,20);
p(51:100,31:50) = ones(50,20);
p(11:40,61:80) = ones(30,20);
p(71:80,61:100) = ones(10,40);
p(51:60,11:30) = ones(10,20);
map = binaryOccupancyMap(p);

% Simulate lidar sensor. Use default detection angle of [-pi pi]
lidar = rangeSensor;
% Set min and max values of the detectable range of the sensor in meters
lidar.Range = [0 50];
% 
% % Generate lidar readings
% [ranges,angles] = lidar(pose,map);
% scan = lidarScan(ranges,angles);
% plot(scan)

%% Trajectory
% Arrival, Waypoints, Orientation
constraints = [0,    0,0,0,    0,0,0;
               3,    50,20,0,    90,0,0;
               4,    58,15.5,0,  162,0,0;
               5.5,  59.5,0,0    180,0,0];

trajectory = waypointTrajectory(constraints(:,2:4), ...
    'TimeOfArrival',constraints(:,1), ...
    'Orientation',quaternion(constraints(:,5:7),'eulerd','ZYX','frame'));

tInfo = waypointInfo(trajectory);

figure(1)
hold on
show(map)
plot(tInfo.Waypoints(1,1),tInfo.Waypoints(1,2),'b*')
grid on
xlim([0 100])
xlim([0 100])
%daspect([1 1 1])

orient = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,'quaternion');
vel = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);
acc = vel;
angVel = vel;

count = 1;
while ~isDone(trajectory)
   [pos,orient(count),vel(count,:),acc(count,:),angVel(count,:)] = trajectory();

   plot(pos(1),pos(2),'b.')
   
   [ranges,angles] = lidar(pos,map);
   local_scan = lidarScan(ranges,angles);
   global_scan = lidarScan(local_scan.Cartesian + [pos(1),pos(2)]);
   plot(global_scan,'Color','r')
   
   pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
   count = count + 1;
end