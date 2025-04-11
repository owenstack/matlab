% RobotParameters.m
% This script defines all parameters for the robotic vacuum simulation
% These parameters will be saved and used by the simulation scripts

%% Physical Properties
robot_params = struct();

% Basic physical properties
robot_params.radius = 0.15;               % Robot radius in meters
robot_params.height = 0.1;                % Robot height in meters
robot_params.weight = 3.5;                % Weight in kg
robot_params.wheel_diameter = 0.05;       % Wheel diameter in meters
robot_params.wheel_separation = 0.26;     % Distance between wheels in meters

%% Movement Capabilities
robot_params.max_speed = 0.5;             % Maximum linear speed in m/s
robot_params.max_angular_speed = 1.5;     % Maximum rotation speed in rad/s
robot_params.max_acceleration = 0.3;      % Maximum acceleration in m/s²
robot_params.max_angular_accel = 0.8;     % Maximum angular acceleration in rad/s²
robot_params.min_turn_radius = 0.1;       % Minimum turning radius in meters

%% Cleaning Capabilities
robot_params.suction_power = struct();
robot_params.suction_power.carpet = 90;   % Power setting for carpet (0-100%)
robot_params.suction_power.hardwood = 60; % Power setting for hardwood (0-100%)
robot_params.suction_power.tile = 70;     % Power setting for tile (0-100%)
robot_params.suction_power.max = 100;     % Maximum suction power (0-100%)
robot_params.suction_power.eco = 40;      % Eco mode suction power (0-100%)

robot_params.brush = struct();
robot_params.brush.main_speed = struct('carpet', 2500, 'hardwood', 2000, 'tile', 2200); % RPM
robot_params.brush.side_speed = struct('carpet', 1500, 'hardwood', 1200, 'tile', 1300); % RPM
robot_params.brush.width = 0.18;          % Main brush width in meters
robot_params.brush.diameter = 0.05;       % Main brush diameter in meters

robot_params.bin_capacity = 0.5;          % Bin capacity in liters

%% Battery Characteristics
robot_params.battery = struct();
robot_params.battery.capacity = 3200;     % Battery capacity in mAh
robot_params.battery.voltage = 14.4;      % Battery voltage in V
robot_params.battery.runtime = 120;       % Estimated runtime in minutes at normal power
robot_params.battery.charge_time = 180;   % Full charge time in minutes
robot_params.battery.low_threshold = 15;  % Low battery warning threshold (%)
robot_params.battery.critical = 5;        % Critical battery threshold to return to dock (%)

%% Sensor Configurations
robot_params.sensors = struct();

% LiDAR sensor
robot_params.sensors.lidar = struct();
robot_params.sensors.lidar.range = 3.0;   % Maximum detection range in meters
robot_params.sensors.lidar.fov = 360;     % Field of view in degrees
robot_params.sensors.lidar.resolution = 1; % Angular resolution in degrees
robot_params.sensors.lidar.rate = 20;     % Scan rate in Hz
robot_params.sensors.lidar.height = 0.08; % Height from ground in meters

% Camera
robot_params.sensors.camera = struct();
robot_params.sensors.camera.resolution = [640, 480]; % Resolution in pixels
robot_params.sensors.camera.fov = 120;    % Field of view in degrees 
robot_params.sensors.camera.fps = 15;     % Frames per second

% Cliff sensors
robot_params.sensors.cliff = struct();
robot_params.sensors.cliff.count = 4;     % Number of cliff sensors
robot_params.sensors.cliff.threshold = 0.05; % Detection threshold in meters
robot_params.sensors.cliff.positions = [  % Positions relative to center [x,y,z]
    0.15, 0.15, -0.02;   % Front-right
    0.15, -0.15, -0.02;  % Front-left
    -0.15, 0.15, -0.02;  % Rear-right
    -0.15, -0.15, -0.02  % Rear-left
];

% Bump sensors
robot_params.sensors.bump = struct();
robot_params.sensors.bump.count = 3;      % Number of bump sensors (front, left, right)
robot_params.sensors.bump.sensitivity = 0.8; % Sensitivity (0-1)

% Floor type sensor
robot_params.sensors.floor_type = struct();
robot_params.sensors.floor_type.types = {'carpet', 'hardwood', 'tile', 'unknown'};
robot_params.sensors.floor_type.accuracy = 0.9; % Accuracy of floor type detection

% Dirt detection sensor
robot_params.sensors.dirt = struct();
robot_params.sensors.dirt.sensitivity = 0.8; % Detection sensitivity (0-1)
robot_params.sensors.dirt.levels = 10;    % Number of dirt level gradations

% IMU (Inertial Measurement Unit)
robot_params.sensors.imu = struct();
robot_params.sensors.imu.accel_range = 2; % Accelerometer range in g
robot_params.sensors.imu.gyro_range = 250; % Gyroscope range in deg/s

% Wheel encoders
robot_params.sensors.encoders = struct();
robot_params.sensors.encoders.ticks_per_rev = 1200; % Encoder resolution
robot_params.sensors.encoders.precision = 0.002;   % Position precision in meters

%% Navigation Parameters
robot_params.navigation = struct();
robot_params.navigation.map_resolution = 0.05; % Map resolution in meters/cell
robot_params.navigation.min_obstacle_dist = 0.1; % Minimum distance to obstacles in meters
robot_params.navigation.coverage_speed = 0.2; % Speed during coverage in m/s
robot_params.navigation.return_speed = 0.3;  % Speed during return to dock in m/s
robot_params.navigation.cleaning_patterns = {'spiral', 'snake', 'wall_follow', 'random'};
robot_params.navigation.default_pattern = 'spiral';
robot_params.navigation.room_detection = true; % Enable automatic room detection

%% Performance Metrics
robot_params.performance = struct();
robot_params.performance.coverage_rate = 0.4; % Area coverage rate in m²/min
robot_params.performance.energy_efficiency = 25; % Area per energy unit m²/Wh
robot_params.performance.expected_bin_fill_rate = 0.2; % Bin fill rate per m² (%)
robot_params.performance.recommended_recharge = 20; % Recommended recharge percentage

%% Save the parameters to a MAT file
save('robot_parameters.mat', 'robot_params');