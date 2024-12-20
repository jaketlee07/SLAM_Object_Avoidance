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
    lidarMsg = receive(lidarSub, 10); % Wait up to 10 seconds for data
    ranges = double(lidarMsg.Ranges);
    
    % Generate angles based on the size of ranges
    numRanges = numel(ranges);
    angles = linspace(double(lidarMsg.AngleMin), double(lidarMsg.AngleMax), numRanges);

    % Validate and clean ranges
    % validIndices = (ranges > lidarMsg.RangeMin) & (ranges < lidarMsg.RangeMax) & ~isnan(ranges) & ~isinf(ranges);
    validIndices = (ranges > 0) & (ranges < lidarMsg.RangeMax) & ~isnan(ranges) & ~isinf(ranges);
    ranges = ranges(validIndices);
    angles = angles(validIndices);
    
    % ranges = abs(ranges(validIndices));
    % angles = abs(angles(validIndices));
    
    % ranges = abs(ranges);
    % angles = abs(angles);

    % Check for valid data
    if isempty(ranges) || isempty(angles)
        disp('No valid LIDAR data. Skipping...');
        continue;
    end

    % Ensure sizes match
    if length(ranges) ~= length(angles)
        disp('Mismatch between Ranges and Angles. Skipping...');
        continue;
    end

    % Create lidarScan object
    try
        scan = lidarScan(ranges, angles);
    catch ME
        disp(['Error creating lidarScan: ' ME.message]);
        continue;
    end

    % Add scan to SLAM object
    [isScanAccepted, loopClosureInfo] = addScan(slamObj, scan);

    % Visualization
    clf;
    show(slamObj);
    title('Live SLAM Mapping with ROS and LIDAR');
    drawnow;

    % Check for loop closure
    if ~isempty(loopClosureInfo.LoopClosure)
        disp('Loop closure detected.');
    end

    pause(0.1); % Adjust for loop timing if needed
end

%% Shut Down ROS Connection
rosshutdown;
