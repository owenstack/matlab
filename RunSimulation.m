% Run the complete robotic vacuum simulation based on PEAS framework
% This script sets up the environment, creates the robot, and executes the intelligent controller

% Clear workspace and command window
clear;
clc;

disp('=================================================');
disp('INTELLIGENT ROBOTIC VACUUM SIMULATION (PEAS)');
disp('=================================================');

try
    %% 1. Create the simulation world
    disp('Creating 3D simulation world...');
    world = sim3d.World();
    disp('World created successfully.');
    
    %% 2. Load robot parameters
    disp('Loading robot parameters...');
    try
        % Load the parameters file with error handling for different potential paths
        % Try multiple possible locations for the parameters file
        if exist('robot_parameters.mat', 'file')
            load('robot_parameters.mat');
            disp('Robot parameters loaded successfully from current directory.');
        elseif exist('/home/owenstack/repos/matlab/matlab_scripts/robot_parameters.mat', 'file')
            load('/home/owenstack/repos/matlab/matlab_scripts/robot_parameters.mat');
            disp('Robot parameters loaded successfully from absolute path.');
        else
            % If file can't be found, check subdirectories
            files = dir('**/robot_parameters.mat');
            if ~isempty(files)
                load(fullfile(files(1).folder, files(1).name));
                disp(['Robot parameters loaded from: ', fullfile(files(1).folder, files(1).name)]);
            else
                error('Could not locate robot_parameters.mat');
            end
        end
        
        % Display loaded parameters
        disp('Available parameters:');
        whos
    catch ME
        disp('Error loading robot parameters:');
        disp(ME.message);
        disp('Continuing with default parameters...');
        
        % Set default parameters in case loading fails
        % These would typically come from the .mat file
        robot_params = struct();
        robot_params.radius = 0.2;
        robot_params.height = 0.1;
        robot_params.max_speed = 0.5;
        robot_params.max_battery = 100;
        robot_params.suction_power = struct('carpet', 90, 'hardwood', 60, 'tile', 70);
    end
    
    %% 3. Set up the environment based on PEAS description
    disp('Setting up environment based on PEAS description...');
    
    % Create floor with different surfaces
    floor_size = [10, 10];  % 10m x 10m room
    floor = sim3d.Actor(world, 'Name', 'Floor', 'Shape', 'plane', 'Size', floor_size);
    floor.Translation = [0, 0, 0];
    floor.Color = [0.9, 0.9, 0.9]; % Light gray for base floor
    
    % Define different floor types (sections)
    carpet_area = sim3d.Actor(world, 'Name', 'Carpet', 'Shape', 'box', 'Size', [3, 3, 0.01]);
    carpet_area.Translation = [2, 2, 0.005];
    carpet_area.Color = [0.7, 0.3, 0.3]; % Reddish for carpet
    
    hardwood_area = sim3d.Actor(world, 'Name', 'Hardwood', 'Shape', 'box', 'Size', [3, 3, 0.01]);
    hardwood_area.Translation = [-2, 2, 0.005];
    hardwood_area.Color = [0.6, 0.4, 0.2]; % Brown for hardwood
    
    tile_area = sim3d.Actor(world, 'Name', 'Tile', 'Shape', 'box', 'Size', [3, 3, 0.01]);
    tile_area.Translation = [-2, -2, 0.005];
    tile_area.Color = [0.8, 0.8, 0.8]; % Light gray for tile
    
    % Add common household obstacles
    % A table in the center
    table = sim3d.Actor(world, 'Name', 'Table', 'Shape', 'box', 'Size', [1.5, 1.5, 0.7]);
    table.Translation = [0, 0, 0.35];
    table.Color = [0.4, 0.3, 0.2]; % Dark wood color
    
    % Add table legs
    leg_positions = [
        0.6, 0.6, 0;
        0.6, -0.6, 0;
        -0.6, 0.6, 0;
        -0.6, -0.6, 0
    ];
    
    for i = 1:4
        table_leg = sim3d.Actor(world, 'Name', ['TableLeg', num2str(i)], 'Shape', 'cylinder', 'Size', [0.05, 0.7]);
        table_leg.Translation = [leg_positions(i,1), leg_positions(i,2), 0.35];
        table_leg.Color = [0.4, 0.3, 0.2]; % Match table color
    end
    
    % Add chair legs as obstacles
    chair_positions = [
        3, 3, 0;
        3, -3, 0;
        -3, 3, 0;
        -3, -3, 0
    ];
    
    for i = 1:4
        % Create chair base
        chair = sim3d.Actor(world, 'Name', ['Chair', num2str(i)], 'Shape', 'box', 'Size', [0.5, 0.5, 0.45]);
        chair.Translation = [chair_positions(i,1), chair_positions(i,2), 0.225];
        chair.Color = [0.3, 0.3, 0.3]; % Gray for chair
        
        % Add chair legs
        for j = 1:4
            angle = (j-1) * pi/2;
            x_offset = 0.2 * cos(angle);
            y_offset = 0.2 * sin(angle);
            
            chair_leg = sim3d.Actor(world, 'Name', ['Chair', num2str(i), 'Leg', num2str(j)], 'Shape', 'cylinder', 'Size', [0.02, 0.45]);
            chair_leg.Translation = [chair_positions(i,1) + x_offset, chair_positions(i,2) + y_offset, 0.225];
            chair_leg.Color = [0.2, 0.2, 0.2]; % Darker gray for legs
        end
    end
    
    % Add a "cliff" (stairs)
    cliff = sim3d.Actor(world, 'Name', 'Stairs', 'Shape', 'box', 'Size', [2, 1, 0.2]);
    cliff.Translation = [4, -4, 0.1];
    cliff.Color = [0.5, 0.5, 0.5]; % Gray for stairs
    
    % Add charging dock
    dock = sim3d.Actor(world, 'Name', 'ChargingDock', 'Shape', 'box', 'Size', [0.3, 0.3, 0.1]);
    dock.Translation = [-4, -4, 0.05];
    dock.Color = [0.2, 0.6, 0.2]; % Green for charging dock
    
    % Add a pet (simplified as a moving obstacle)
    pet = sim3d.Actor(world, 'Name', 'Pet', 'Shape', 'sphere', 'Size', 0.2);
    pet.Translation = [1, -1, 0.2];
    pet.Color = [0.7, 0.6, 0.4]; % Beige color for pet
    
    % Add some cables on the floor as challenging obstacles
    cable1 = sim3d.Actor(world, 'Name', 'Cable1', 'Shape', 'cylinder', 'Size', [0.02, 3]);
    cable1.Translation = [1, 4, 0.01];
    cable1.Rotation = [0, 90, 0]; % Lay the cable flat on the ground
    cable1.Color = [0.1, 0.1, 0.1]; % Black for cable
    
    cable2 = sim3d.Actor(world, 'Name', 'Cable2', 'Shape', 'cylinder', 'Size', [0.02, 2.5]);
    cable2.Translation = [-1, -3.5, 0.01];
    cable2.Rotation = [0, 45, 0]; % Angled cable
    cable2.Color = [0.7, 0.7, 0.7]; % Gray for cable
    
    % Add small objects (toys, etc.) as obstacles
    for i = 1:5
        toy = sim3d.Actor(world, 'Name', ['SmallObject', num2str(i)], 'Shape', 'sphere', 'Size', 0.1);
        toy.Translation = [4*rand()-2, 4*rand()-2, 0.05];
        toy.Color = [rand(), rand(), rand()]; % Random colors
    end
    
    % Add area rugs (potential mobility challenges)
    rug = sim3d.Actor(world, 'Name', 'AreaRug', 'Shape', 'box', 'Size', [2, 1.5, 0.02]);
    rug.Translation = [0, -2, 0.01];
    rug.Color = [0.2, 0.2, 0.6]; % Blue for rug
    
    disp('Environment setup complete.');
    
    %% 4. Define the path to robot's 3D model file
    meshPath = 'resources/robovacum.fbx';
    disp(['Attempting to load mesh from: ', meshPath]);
    
    %% 5. Create the robot actor within the world
    disp('Creating robot actor...');
    try
        % First attempt with specified mesh
        robot = sim3d.Actor(world, 'Name', 'Smart Vacuum', 'Mesh', meshPath);
        disp('Robot actor created successfully with custom mesh.');
    catch ME
        % Fallback to primitive shape if mesh loading fails
        disp(['Failed to load mesh: ', ME.message]);
        disp('Creating robot using primitive shapes instead...');
        
        % Create robot base
        robot = sim3d.Actor(world, 'Name', 'Smart Vacuum', 'Shape', 'cylinder', 'Size', [0.3, 0.1]);
        robot.Color = [0.2, 0.2, 0.2]; % Dark gray for robot base
        
        % Add top cover with different color
        robot_top = sim3d.Actor(world, 'Name', 'VacuumTop', 'Shape', 'cylinder', 'Size', [0.28, 0.03]);
        robot_top.Translation = [0, 0, 0.065];
        robot_top.Color = [0.4, 0.4, 0.8]; % Blue-gray for top
        robot_top.Parent = robot; % Attach to main robot
        
        % Add sensor dome on top
        robot_dome = sim3d.Actor(world, 'Name', 'SensorDome', 'Shape', 'sphere', 'Size', 0.08);
        robot_dome.Translation = [0, 0, 0.11];
        robot_dome.Color = [0.3, 0.3, 0.3]; % Dark gray for sensor dome
        robot_dome.Parent = robot; % Attach to main robot
        
        disp('Robot created with primitive shapes.');
    end
    
    % Set initial position/orientation
    initialTranslation = [0, 0, 0.05]; % Slightly above ground to avoid collision issues
    initialRotation = [0, 0, 0];       % No initial rotation
    robot.Translation = initialTranslation;
    robot.Rotation = initialRotation;
    
    disp(['Set initial position to: [', num2str(initialTranslation), ']']);
    disp(['Set initial rotation to: [', num2str(initialRotation), ']']);
    
    %% 6. Setup the vacuum cleaner's sensor systems based on PEAS
    disp('Setting up sensor systems...');
    
    % Add LiDAR sensor (visualization)
    lidar_viz = sim3d.Actor(world, 'Name', 'LidarSensor', 'Shape', 'cylinder', 'Size', [0.15, 0.03]);
    lidar_viz.Translation = [0, 0, 0.1]; % Position on top of vacuum
    lidar_viz.Color = [0.1, 0.1, 0.1]; % Black for sensor
    lidar_viz.Parent = robot; % Attach to robot
    
    % Add cliff sensors (4 at corners)
    cliff_sensor_positions = [
        0.15, 0.15, -0.02;  % Front-right
        0.15, -0.15, -0.02; % Front-left
        -0.15, 0.15, -0.02; % Rear-right
        -0.15, -0.15, -0.02 % Rear-left
    ];
    
    for i = 1:4
        cliff_sensor = sim3d.Actor(world, 'Name', ['CliffSensor', num2str(i)], 'Shape', 'sphere', 'Size', 0.02);
        cliff_sensor.Translation = cliff_sensor_positions(i,:);
        cliff_sensor.Color = [1, 0, 0]; % Red for sensor
        cliff_sensor.Parent = robot; % Attach to robot
    end
    
    % Add bump sensor (front semicircle)
    bump_sensor = sim3d.Actor(world, 'Name', 'BumpSensor', 'Shape', 'box', 'Size', [0.3, 0.04, 0.03]);
    bump_sensor.Translation = [0.15, 0, 0.03];
    bump_sensor.Color = [0.9, 0.7, 0]; % Orange for bump sensor
    bump_sensor.Parent = robot; % Attach to robot
    
    % Add camera visualization (for object recognition)
    camera = sim3d.Actor(world, 'Name', 'Camera', 'Shape', 'sphere', 'Size', 0.03);
    camera.Translation = [0.15, 0, 0.08];
    camera.Color = [0, 0, 0]; % Black for camera
    camera.Parent = robot; % Attach to robot
    
    % Add wheels (for visualization)
    wheel_left = sim3d.Actor(world, 'Name', 'LeftWheel', 'Shape', 'cylinder', 'Size', [0.05, 0.02]);
    wheel_left.Translation = [0, 0.16, 0];
    wheel_left.Rotation = [0, 0, 90]; % Rotate to proper orientation
    wheel_left.Color = [0.1, 0.1, 0.1]; % Black for wheels
    wheel_left.Parent = robot; % Attach to robot
    
    wheel_right = sim3d.Actor(world, 'Name', 'RightWheel', 'Shape', 'cylinder', 'Size', [0.05, 0.02]);
    wheel_right.Translation = [0, -0.16, 0];
    wheel_right.Rotation = [0, 0, 90]; % Rotate to proper orientation
    wheel_right.Color = [0.1, 0.1, 0.1]; % Black for wheels
    wheel_right.Parent = robot; % Attach to robot
    
    % Add brush visualization
    main_brush = sim3d.Actor(world, 'Name', 'MainBrush', 'Shape', 'cylinder', 'Size', [0.2, 0.03]);
    main_brush.Translation = [0, 0, -0.03];
    main_brush.Rotation = [90, 0, 0]; % Rotate to proper orientation
    main_brush.Color = [0.5, 0.5, 0.5]; % Gray for brush
    main_brush.Parent = robot; % Attach to robot
    
    disp('Sensor systems setup complete.');
    
    %% 7. Initialize the vacuum controller
    disp('Initializing vacuum controller...');
    controller = VacuumController(robot, world);
    
    %% 8. View the world
    disp('Opening 3D view...');
    view(world);
    
    %% 9. Run the simulation
    disp('Starting simulation...');
    disp('Press Ctrl+C to stop the simulation at any time');
    
    % Ask user if they want to run the full cleaning cycle or manual demo
    choice = input('Select mode (1 for full cleaning cycle, 2 for manual demo): ');
    
    if choice == 1
        % Run full cleaning cycle with intelligent controller
        controller.run_cleaning_cycle();
    else
        % Manual demo mode - simple animation for demonstration
        disp('Running manual demo mode (30 seconds)...');
        
        % Simple motion pattern for demonstration
        start_time = tic;
        elapsed = 0;
        
        % Define a more complex path to demonstrate capabilities
        while elapsed < 30 % Run for 30 seconds
            % Calculate time-based position
            t = elapsed;
            
            % First demonstrate spiral pattern (0-10s)
            if t < 10
                radius = 0.1 + 0.15 * t;
                angle = t * 3;
                x = radius * cos(angle);
                y = radius * sin(angle);
                theta = angle * 180/pi;
                
                % Simulate dirt detection periodically
                if mod(t, 2) < 0.1
                    disp('Dirt detected! Increasing suction power...');
                end
            % Then demonstrate wall following (10-20s)
            elseif t < 20
                wall_t = t - 10;
                % Rectangle path
                if wall_t < 2.5
                    x = 3.0;
                    y = -2.0 + 1.6 * wall_t;
                    theta = 90;
                elseif wall_t < 5
                    x = 3.0 - 1.6 * (wall_t - 2.5);
                    y = 2.0;
                    theta = 180;
                elseif wall_t < 7.5
                    x = -1.0;
                    y = 2.0 - 1.6 * (wall_t - 5);
                    theta = 270;
                else
                    x = -1.0 + 1.6 * (wall_t - 7.5);
                    y = -2.0;
                    theta = 0;
                end
                
                % Wall detection messages
                if mod(wall_t, 2.5) < 0.1
                    disp('Wall detected! Adjusting path...');
                end
            % Finally demonstrate obstacle avoidance (20-30s)
            else
                avoid_t = t - 20;
                % Head toward obstacle, then avoid
                if avoid_t < 3
                    x = -1.0 + avoid_t * 0.67;
                    y = avoid_t * 0.33;
                    theta = atan2(y, x) * 180/pi;
                elseif avoid_t < 3.5
                    % Detect obstacle
                    x = 1.0;
                    y = 1.0;
                    theta = 45;
                    if avoid_t < 3.1
                        disp('Obstacle detected! Calculating avoidance path...');
                    end
                elseif avoid_t < 6
                    % Avoidance maneuver
                    circle_t = avoid_t - 3.5;
                    angle = 45 + circle_t * 180;
                    radius = 1.0;
                    x = 1.0 + radius * cos(angle * pi/180);
                    y = 1.0 + radius * sin(angle * pi/180);
                    theta = angle + 180;
                else
                    % Resume normal path
                    path_t = avoid_t - 6;
                    x = 1.0 - path_t * 0.5;
                    y = 1.0 - path_t * 0.5;
                    theta = 225;
                    
                    % Detect floor type
                    if x < -0.5 && y < -0.5
                        disp('Tile floor detected! Adjusting cleaning parameters...');
                    elseif x > 0.5 && y > 0.5
                        disp('Carpet detected! Increasing brush speed and suction...');
                    end
                end
            end
            
            % Update robot position and orientation
            robot.Translation = [x, y, 0.05];
            robot.Rotation = [theta, 0, 0];
            
            % Also animate the pet (random movements)
            if mod(elapsed, 1) < 0.1
                pet.Translation = [1 + 0.5*randn, -1 + 0.5*randn, 0.2];
            end
            
            % Update elapsed time
            pause(0.1);
            elapsed = toc(start_time);
        end
        
        disp('Demo complete.');
    end
    
    %% 10. Final reporting
    disp('Simulation complete.');
    disp(''); 
    disp('PEAS Framework Implementation:');
    disp('-----------------------------');
    disp('Performance Measure: The simulation tracked cleaning coverage,');
    disp('                     dirt collection, energy usage, and navigation metrics.');
    disp('Environment:         Simulated multiple floor types, obstacles,');
    disp('                     and dynamic elements like pets.');
    disp('Actuators:           Implemented drive system, brush system,');
    disp('                     and suction control in the controller.');
    disp('Sensors:             Simulated LiDAR, cliff sensors, bump sensors,');
    disp('                     floor type detection, and dirt detection.');
    
catch ME
    disp('Error in simulation:');
    disp(ME.message);
    
    % Provide more detailed error information
    if strcmp(ME.identifier, 'sim3d:CommandProcessor:ActorAssetNotFound')
        disp('Common Issue: Check that the mesh path is correct and the file exists.');
        disp(['Current Folder: ', pwd]);
        disp(['Mesh path used: ', meshPath]);
        
        % List files in the resources directory to help diagnose
        try
            dirContents = dir('resources');
            disp('Contents of resources directory:');
            disp({dirContents.name});
        catch
            disp('Could not access resources directory');
        end
        
    elseif contains(ME.message, 'license')
        disp('Common Issue: Ensure you have the required Simulink 3D Animation license.')
    else
        % Display full error stack for debugging
        disp('Error stack:');
        disp(getReport(ME));
    end
end

% Add a cleanup to close figures if needed
% close all;