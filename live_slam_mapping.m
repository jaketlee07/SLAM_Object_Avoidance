% live_slam_mapping.m
% This script retrieves LiDAR and IMU data from a Flask server, processes
% it using SLAM, and visualizes the live map in real-time.

% Robot Flask Server IP
robot_ip = 'http://192.168.5.205:5000';  % Replace with your robot's IP
url = strcat(robot_ip, '/get_combined_data');

% Initialize SLAM Object
maxRange = 8; % Maximum range of the LiDAR in meters
mapResolution = 20; % Resolution of the map in cells per meter
slamObj = lidarSLAM(mapResolution, maxRange);

% SLAM Parameters
slamObj.LoopClosureThreshold = 210; % Adjust as needed
slamObj.LoopClosureSearchRadius = 2; % Adjust as needed

% Visualization Setup
figure;
title('Live SLAM Map');
hold on;

% Variables for Live Mapping
lastScan = [];
robotPose = [];

while true
    try
        % Fetch data from Flask server
        data = webread(url);

        % Process LiDAR data
        lidarData = data.lidar;
        angleMin = lidarData.angle_min;
        angleMax = lidarData.angle_max;
        angleIncrement = lidarData.angle_increment;
        ranges = lidarData.ranges;
        
        % Ignore invalid range values (NaN or Inf)
        validIdx = ~isnan(ranges) & ~isinf(ranges);
        angles = angleMin:angleIncrement:angleMax;
        ranges = ranges(validIdx);
        angles = angles(validIdx);

        % Convert LiDAR data into a MATLAB lidarScan object
        scan = lidarScan(ranges, angles);

        % Add scan to SLAM and update map
        [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj, scan);

        if isScanAccepted
            % Extract the robot's estimated pose
            robotPose = getRobotPose(slamObj);

            % Visualize the updated map
            show(slamObj);
            title('Live SLAM Map');
            xlabel('X (m)');
            ylabel('Y (m)');
            drawnow;
        end

        % Pause for live data streaming
        pause(0.1); % Adjust for desired refresh rate
    catch ME
        % Handle errors (e.g., network timeout or invalid data)
        disp('Error fetching or processing data:');
        disp(ME.message);
        pause(1); % Retry after 1 second
    end
end
