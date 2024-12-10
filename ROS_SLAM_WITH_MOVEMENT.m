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

%% Create TCP/IP Client for Motion Control
limoAddress = '192.168.0.109';  % Set this to the Limo's IP address
limoPort = 12345;  % Do not change
tcpipClient = tcpip(limoAddress, limoPort, 'NetworkRole', 'client');
fopen(tcpipClient);

%% Main SLAM and Navigation Loop
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
    
    % Get current pose from SLAM object
    currentPose = slamObj.PoseGraph.nodes(end).Pose;
    
    % Define goal position (you can set this dynamically)
    goalPosition = [5, 5];  % Example goal [x, y]
    
    % Plan path to goal
    occGrid = occupancyMap(slamObj.getOccupancyGrid);
    prm = mobileRobotPRM(occGrid);
    path = findpath(prm, currentPose(1:2), goalPosition);
    
    if ~isempty(path)
        % Follow the path
        for i = 2:size(path, 1)
            targetPoint = path(i, :);
            
            % Calculate desired linear velocity and steering angle
            dx = targetPoint(1) - currentPose(1);
            dy = targetPoint(2) - currentPose(2);
            distance = sqrt(dx^2 + dy^2);
            angle = atan2(dy, dx) - currentPose(3);
            
            linearVelocity = min(0.2, distance);  % Limit to max speed of 0.2 m/s
            steeringAngle = angle;
            
            % Send command to LIMO
            command = sprintf('%f,%f', linearVelocity, steeringAngle);
            fwrite(tcpipClient, command);
            
            % Wait for execution
            pause(0.3);
            
            % Update visualization
            clf;
            show(slamObj);
            hold on;
            plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
            plot(targetPoint(1), targetPoint(2), 'go', 'MarkerSize', 10);
            hold off;
            title('Live SLAM Mapping and Navigation');
            drawnow;
        end
    else
        disp('No path found to goal');
    end
    
    % Check if goal is reached
    if norm(currentPose(1:2) - goalPosition) < 0.1
        disp('Goal reached!');
        break;
    end
end

%% Shut Down
fclose(tcpipClient);
delete(tcpipClient);
clear tcpipClient;
rosshutdown;
