% Clear workspace and command window
clear;
clc;

%% 1. Create the simulation world object
disp('Creating 3D simulation world...');
world = sim3d.World();
disp('World created.');

%% 2. Load robot parameters
disp('Loading robot parameters...');
try
    % Load the parameters file
    load('robot_parameters.mat');
    disp('Robot parameters loaded successfully.');
    
    % Display loaded parameters for verification
    disp('Available parameters:');
    whos
catch ME
    disp('Error loading robot parameters:');
    disp(ME.message);
    disp(['Current working directory: ', pwd]);
    disp('Please ensure robot_parameters.mat is in the correct path.');
end

%% 3. Define the path to robot's 3D model file
meshPath = 'resources/robovacum.fbx';
disp(['Attempting to load mesh from: ', meshPath]);

%% 4. Create the robot actor within the world
disp('Creating robot actor...');
try
    % Create the robot actor - ensuring proper Name-Value pair syntax
    robot = sim3d.Actor(world, 'Name', 'Smart Vacuum', 'Mesh', meshPath);
    disp('Robot actor ''Smart Vacuum'' created successfully.');
    
    % Set initial position/orientation
    initialTranslation = [0, 0, 0.05]; % Slightly above ground to avoid collision issues
    initialRotation = [0, 0, 0];       % No initial rotation
    robot.Translation = initialTranslation;
    robot.Rotation = initialRotation;
    
    disp(['Set initial position to: [', num2str(initialTranslation), ']']);
    disp(['Set initial rotation to: [', num2str(initialRotation), ']']);
    
    %% 5. Setup Environment based on PEAS
    disp('Setting up environment based on PEAS description...');
    
    % Create floor with different surfaces
    floor_size = [5, 5];  % 5m x 5m room
    floor = sim3d.Actor(world, 'Name', 'Floor', 'Shape', 'plane', 'Size', floor_size);
    floor.Translation = [0, 0, 0];
    
    % Define different floor types (sections)
    % We'll create visual indicators for different floor types
    carpet_area = sim3d.Actor(world, 'Name', 'Carpet', 'Shape', 'box', 'Size', [2, 2, 0.01]);
    carpet_area.Translation = [1.5, 1.5, 0.005];
    carpet_area.Color = [0.7, 0.3, 0.3]; % Reddish for carpet
    
    hardwood_area = sim3d.Actor(world, 'Name', 'Hardwood', 'Shape', 'box', 'Size', [2, 2, 0.01]);
    hardwood_area.Translation = [-1.5, 1.5, 0.005];
    hardwood_area.Color = [0.6, 0.4, 0.2]; % Brown for hardwood
    
    tile_area = sim3d.Actor(world, 'Name', 'Tile', 'Shape', 'box', 'Size', [2, 2, 0.01]);
    tile_area.Translation = [-1.5, -1.5, 0.005];
    tile_area.Color = [0.8, 0.8, 0.8]; % Light gray for tile
    
    % Add obstacles (furniture)
    table = sim3d.Actor(world, 'Name', 'Table', 'Shape', 'box', 'Size', [1, 1, 0.6]);
    table.Translation = [0, 0, 0.3];
    
    % Add chair legs as obstacles
    for i = 1:4
        angle = (i-1) * pi/2;
        x = 0.7 * cos(angle);
        y = 0.7 * sin(angle);
        chair_leg = sim3d.Actor(world, 'Name', ['ChairLeg', num2str(i)], 'Shape', 'cylinder', 'Size', [0.05, 0.4]);
        chair_leg.Translation = [x, y, 0.2];
    end
    
    % Add a "cliff" (stairs)
    cliff = sim3d.Actor(world, 'Name', 'Stairs', 'Shape', 'box', 'Size', [1, 0.5, 0.1]);
    cliff.Translation = [2, -2, 0.05];
    
    % Add charging dock
    dock = sim3d.Actor(world, 'Name', 'ChargingDock', 'Shape', 'box', 'Size', [0.3, 0.3, 0.1]);
    dock.Translation = [-2, -2, 0.05];
    dock.Color = [0.2, 0.6, 0.2]; % Green for charging dock
    
    % Add a pet (simplified as a moving obstacle)
    pet = sim3d.Actor(world, 'Name', 'Pet', 'Shape', 'sphere', 'Size', 0.2);
    pet.Translation = [1, -1, 0.1];
    pet.Color = [0.5, 0.5, 0.7]; % Bluish for pet
    
    % Add some cables on the floor
    cable = sim3d.Actor(world, 'Name', 'Cable', 'Shape', 'cylinder', 'Size', [0.02, 2]);
    cable.Translation = [0, 2, 0.01];
    cable.Rotation = [0, 90, 0]; % Lay the cable flat on the ground
    cable.Color = [0.1, 0.1, 0.1]; % Black for cable
    
    disp('Environment setup complete.');
    
    %% 6. Setup the vacuum cleaner's sensor system based on PEAS
    % Note: This would require creating sensor objects and attaching them to the robot
    % The implementation details depend on your specific sim3d library capabilities
    
    % For example, we could simulate:
    % - LiDAR or camera for mapping and obstacle detection
    % - Cliff sensors for detecting stairs
    % - Bump sensors for collision detection
    % - Floor type detection sensors
    % - Battery level simulation
    
    disp('Setting up sensor systems...');
    % Add LiDAR sensor (simplified as visualization)
    lidar_viz = sim3d.Actor(world, 'Name', 'LidarSensor', 'Shape', 'cylinder', 'Size', [0.1, 0.05]);
    lidar_viz.Translation = [0, 0, 0.1]; % Position on top of vacuum
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
    
    % Add bump sensor (front)
    bump_sensor = sim3d.Actor(world, 'Name', 'BumpSensor', 'Shape', 'box', 'Size', [0.2, 0.03, 0.03]);
    bump_sensor.Translation = [0.18, 0, 0.03];
    bump_sensor.Color = [0.9, 0.7, 0]; % Orange for bump sensor
    bump_sensor.Parent = robot; % Attach to robot
    
    disp('Sensor systems setup complete.');
    
    %% 7. View the world and actor
    disp('Opening 3D view...');
    view(world);
    
    %% 8. Optional: Animation
    disp('Press Ctrl+C to stop the simulation');
    for t = 0:0.1:10
        % Simple circular motion for demonstration
        robot.Translation = [cos(t), sin(t), 0.05];
        robot.Rotation = [t*180/pi, 0, 0]; % Rotate based on direction
        
        % Update pet position (random movement)
        if mod(t, 1) < 0.1
            pet.Translation = [1 + 0.5*randn, -1 + 0.5*randn, 0.1];
        end
        
        pause(0.1); % Control animation speed
    end
    
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