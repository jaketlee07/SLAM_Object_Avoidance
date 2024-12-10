%% Live Lidar SLAM with Factor Graph Optimization
% This script performs real-time SLAM using live LIDAR data and builds a 2D map.

%% Parameters and Initialization
robotIP = '192.168.5.205'; % Replace with your robot's IP address
port = 5000; % Flask server port
maxLidarRange = 12.0; % Maximum range of the LIDAR
mapResolution = 20; % Cells per meter
loopClosureThreshold = 210; % Loop closure threshold
loopClosureSearchRadius = 8; % Loop closure search radius (meters)

% Create LIDAR SLAM object
slamObj = lidarSLAM(mapResolution, maxLidarRange);
slamObj.LoopClosureThreshold = loopClosureThreshold;
slamObj.LoopClosureSearchRadius = loopClosureSearchRadius;

% Initialize variables
firstTimeLCDetected = false;
scanCount = 0;

%% Main Loop for Real-Time Processing
while true
    % Fetch LIDAR and IMU data from Flask endpoint
    data = webread(sprintf('http://%s:%d/get_combined_data', robotIP, port));
    
    if isempty(data) || ~isfield(data, 'lidar') || ~isfield(data, 'imu')
        disp('Waiting for valid data...');
        pause(0.1);
        continue;
    end

    % Extract LIDAR scan data
    ranges = data.lidar.ranges;
    angleMin = data.lidar.angle_min;
    angleMax = data.lidar.angle_max;
    angleIncrement = data.lidar.angle_increment;
    
    % Validate ranges and angles
    if isempty(ranges) || isempty(angleMin) || isempty(angleMax) || isempty(angleIncrement)
        disp('Invalid LIDAR data. Skipping this scan...');
        continue;
    end

    % Construct angles array
    angles = angleMin:angleIncrement:(angleMin + angleIncrement * (length(ranges) - 1));
    if length(angles) ~= length(ranges)
        disp(['Mismatch between ranges and angles. Adjusting to minimum length. Ranges: ' num2str(length(ranges)) ', Angles: ' num2str(length(angles))]);
        minLength = min(length(angles), length(ranges));
        ranges = ranges(1:minLength);
        angles = angles(1:minLength);
    end

    % Remove invalid range values (e.g., NaN, Inf, or negative values)
    validIndices = (ranges > 0) & (ranges <= maxLidarRange);
    ranges = ranges(validIndices);
    angles = angles(validIndices);

    % Skip if no valid data remains
    if isempty(ranges)
        disp('No valid range data available. Skipping this scan...');
        continue;
    end

    % Construct a lidarScan object
    try
        scan = lidarScan(ranges, angles);
    catch ME
        disp(['Error constructing lidarScan object: ' ME.message]);
        continue;
    end
    
    % Add scan to LIDAR SLAM
    [isScanAccepted, loopClosureInfo] = addScan(slamObj, scan);
    
    if isScanAccepted
        scanCount = scanCount + 1;

        % Plot loop closure if detected
        if ~isempty(loopClosureInfo.LoopClosure)
            disp('Loop closure detected.');
            if ~firstTimeLCDetected
                firstTimeLCDetected = true;
            end
        end
    end

    % Visualization
    clf;
    show(slamObj);
    title('Live SLAM Mapping with LIDAR');
    drawnow;

    pause(0.1); % Adjust loop speed if needed
end
