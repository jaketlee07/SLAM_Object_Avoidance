%% Initialize ROS Connection
rosMasterIP = '192.168.0.195'; % Replace with your robot's ROS master IP
rosMasterPort = 11311; % Default ROS master port
rosinit(rosMasterIP, rosMasterPort);

%% Create ROS Subscriber for LIDAR Data
lidarSub = rossubscriber('/scan', 'sensor_msgs/LaserScan', 'DataFormat', 'struct');

%% Create LIDAR SLAM Object
maxLidarRange = 12.0; % Max range of LIDAR in meters
mapResolution = 20; % Grid cells per meter
slamObj = lidarSLAM(mapResolution, maxLidarRange);
slamObj.LoopClosureThreshold = 210; % Threshold for loop closure
slamObj.LoopClosureSearchRadius = 8; % Search radius for loop closure

%% Main SLAM Loop
figure;
while true
    % Receive LIDAR scan
    scanMsg = receive(lidarSub, 10); % Wait up to 10 seconds for data
    
    % Use rosReadLidarScan to create a lidarScan object
    try
        scan = rosReadLidarScan(scanMsg);
    catch ME
        disp(['Error reading LIDAR scan: ' ME.message]);
        continue;
    end

    % Validate the scan
    if isempty(scan.Ranges) || isempty(scan.Angles)
        disp('No valid LIDAR data. Skipping...');
        continue;
    end

    % Add scan to SLAM object
    [isScanAccepted, loopClosureInfo] = addScan(slamObj, scan);

    % Visualization
    clf;
    show(slamObj);
    title('Live SLAM Mapping with ROS and LIDAR');
    drawnow;

    % % Check for loop closure
    % if isScanAccepted && loopClosureInfo.Score > 0
    %     disp('Loop closure detected.');
    % end

    % pause(0.1); % Adjust for loop timing if needed
end

%% Shut Down ROS Connection
rosshutdown;
