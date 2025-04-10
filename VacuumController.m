% Robotic Vacuum Controller based on PEAS Framework
% This script implements the control logic for the vacuum robot

classdef VacuumController < handle
    % Properties to track robot state and environment
    properties
        % Robot state
        position = [0, 0, 0.05]     % [x, y, z] in meters
        orientation = [0, 0, 0]     % [yaw, pitch, roll] in degrees
        velocity = [0, 0]           % [linear, angular] velocity
        battery_level = 100         % 0-100%
        bin_level = 0               % 0-100%
        current_floor_type = 'unknown'  % carpet, hardwood, tile, etc.
        cleaning_mode = 'normal'    % normal, eco, max, spot
        is_docked = false           % whether robot is at charging dock
        
        % Environment state
        map = []                    % occupancy grid for mapping
        obstacles = []              % detected obstacles
        visited_cells = []          % cells that have been cleaned
        dirt_detected = []          % cells with detected dirt
        room_boundaries = []        % detected room boundaries
        unsafe_areas = []           % detected cliffs, keep-out zones
        dock_location = [-2, -2]    % location of charging dock
        
        % Performance metrics (PEAS)
        area_cleaned = 0            % m²
        coverage_percentage = 0     % % of accessible area
        cleaning_time = 0           % seconds
        energy_used = 0             % watt-hours
        collision_count = 0         % number of collisions
        dirt_collected = 0          % arbitrary units
        
        % References to simulation objects
        robot_actor = []            % handle to robot in simulation
        world = []                  % handle to world in simulation
        sensors = struct()          % struct of sensor handles
    end
    
    % Methods implementing behavior
    methods
        % Constructor
        function obj = VacuumController(robot, world)
            obj.robot_actor = robot;
            obj.world = world;
            obj.initialize_sensors();
            obj.initialize_map();
            disp('Vacuum controller initialized');
        end
        
        % Initialize and configure sensors
        function initialize_sensors(obj)
            disp('Initializing sensors...');
            % In a real implementation, this would configure all sensors
            % For simulation, we'll create placeholders
            
            obj.sensors.lidar = struct('range', 3.0, 'fov', 360);
            obj.sensors.cliff = struct('count', 4, 'threshold', 0.05);
            obj.sensors.bump = struct('front', false, 'left', false, 'right', false);
            obj.sensors.camera = struct('enabled', true, 'resolution', [640, 480]);
            obj.sensors.floor_type = struct('current', 'unknown');
            obj.sensors.imu = struct('acceleration', [0,0,0], 'gyro', [0,0,0]);
            obj.sensors.wheel_encoders = struct('left', 0, 'right', 0);
            obj.sensors.battery = struct('voltage', 14.4, 'current', 0.5);
            obj.sensors.dirt = struct('detected', false, 'level', 0);
            
            disp('Sensors initialized');
        end
        
        % Initialize map of environment
        function initialize_map(obj)
            disp('Initializing map...');
            % Create a simple grid map (10cm resolution)
            map_size = [100, 100]; % 10m x 10m area
            obj.map = zeros(map_size);
            obj.visited_cells = zeros(map_size);
            obj.dirt_detected = zeros(map_size);
            disp('Map initialized');
        end
        
        % Main control loop
        function run_cleaning_cycle(obj)
            disp('Starting cleaning cycle...');
            
            % Initialize cycle
            obj.is_docked = false;
            
            % Main control loop
            while obj.battery_level > 20 && obj.bin_level < 90
                % 1. Update sensor readings
                obj.update_sensor_readings();
                
                % 2. Update map and localization
                obj.update_map();
                
                % 3. Detect floor type and adapt settings
                obj.detect_and_adapt_to_floor_type();
                
                % 4. Plan path based on coverage and dirt detection
                next_move = obj.plan_next_move();
                
                % 5. Execute movement
                obj.move(next_move);
                
                % 6. Check for obstacles and avoid if necessary
                if obj.check_for_obstacles()
                    obj.obstacle_avoidance();
                end
                
                % 7. Check for unsafe areas (cliffs, etc.)
                if obj.check_for_unsafe_areas()
                    obj.safety_maneuver();
                end
                
                % 8. Update cleaning performance metrics
                obj.update_performance_metrics();
                
                % 9. Update simulation display
                obj.update_display();
                
                % 10. Simulate battery and bin usage
                obj.battery_level = obj.battery_level - 0.1;
                if obj.sensors.dirt.detected
                    obj.bin_level = obj.bin_level + 0.5;
                    obj.dirt_collected = obj.dirt_collected + 1;
                end
                
                % Simulate time passing
                pause(0.1);
                obj.cleaning_time = obj.cleaning_time + 0.1;
                
                % Demo: Break after 30 seconds to avoid infinite loop
                if obj.cleaning_time > 30
                    disp('Demo time limit reached');
                    break;
                end
            end
            
            % End of cycle, return to dock
            disp('Cleaning cycle complete, returning to dock...');
            obj.return_to_dock();
            
            % Calculate final metrics
            obj.calculate_final_metrics();
            obj.display_performance_report();
        end
        
        % Update sensor readings from simulation
        function update_sensor_readings(obj)
            % In a real implementation, this would get data from actual sensors
            % For simulation, we'll update based on robot position
            
            % Update position from simulation
            obj.position = obj.robot_actor.Translation;
            obj.orientation = obj.robot_actor.Rotation;
            
            % Simulate LiDAR readings
            % This would detect objects in the simulation environment
            
            % Simulate cliff sensors
            % Check if any of the 4 cliff sensors are above a "hole"
            
            % Simulate bump sensors
            % Check if robot has collided with any objects
            
            % Simulate floor type detection
            % Based on robot position, determine floor type
            x = obj.position(1);
            y = obj.position(2);
            
            if x > 0.5 && x < 2.5 && y > 0.5 && y < 2.5
                obj.current_floor_type = 'carpet';
            elseif x < -0.5 && x > -2.5 && y > 0.5 && y < 2.5
                obj.current_floor_type = 'hardwood';
            elseif x < -0.5 && x > -2.5 && y < -0.5 && y > -2.5
                obj.current_floor_type = 'tile';
            else
                obj.current_floor_type = 'unknown';
            end
            
            % Simulate dirt detection
            % Random dirt detection for demo
            obj.sensors.dirt.detected = (rand() > 0.7);
            obj.sensors.dirt.level = rand() * 10;
            
            % Debug output (comment out for performance)
            % disp(['Position: ', num2str(obj.position), ', Floor: ', obj.current_floor_type]);
        end
        
        % Update internal map based on sensor readings
        function update_map(obj)
            % Convert robot position to map coordinates
            map_x = round(obj.position(1) * 10) + 50;
            map_y = round(obj.position(2) * 10) + 50;
            
            % Stay within bounds
            map_x = max(1, min(map_x, size(obj.map, 1)));
            map_y = max(1, min(map_y, size(obj.map, 2)));
            
            % Mark current cell as visited
            obj.visited_cells(map_x, map_y) = 1;
            
            % If dirt detected, mark on map
            if obj.sensors.dirt.detected
                obj.dirt_detected(map_x, map_y) = obj.sensors.dirt.level;
            end
            
            % Update coverage percentage
            obj.coverage_percentage = 100 * sum(obj.visited_cells(:)) / nnz(obj.map(:) == 0);
        end
        
        % Detect floor type and adapt cleaning parameters
        function detect_and_adapt_to_floor_type(obj)
            % Based on detected floor type, adjust cleaning parameters
            switch obj.current_floor_type
                case 'carpet'
                    % Increase suction, slow down movement
                    obj.velocity = [0.1, 0.5]; % Slower on carpet
                    disp('Carpet detected: Increasing suction power, reducing speed');
                    
                case 'hardwood'
                    % Reduce suction, increase brush action
                    obj.velocity = [0.3, 1.0]; % Faster on hardwood
                    disp('Hardwood detected: Adjusting for hard surface');
                    
                case 'tile'
                    % Medium suction, normal speed
                    obj.velocity = [0.25, 0.8]; % Normal speed on tile
                    disp('Tile detected: Standard cleaning mode');
                    
                otherwise
                    % Default settings
                    obj.velocity = [0.2, 0.7];
            end
        end
        
        % Plan next move based on coverage algorithm
        function next_move = plan_next_move(obj)
            % Simplified path planning - spiral pattern
            % In a real implementation, this would use a coverage algorithm
            % like spiral, snake, or random bounce with memory
            
            % For demo, just move in a spiral pattern
            t = obj.cleaning_time;
            radius = 0.1 + 0.05 * t;
            angle = t * 2;
            
            next_move.x = radius * cos(angle);
            next_move.y = radius * sin(angle);
            next_move.theta = angle * 180/pi;
            
            % If we detect a lot of dirt, plan to stay in this area longer
            if obj.sensors.dirt.level > 5
                next_move.speed_factor = 0.5; % Slow down in dirty areas
                disp('High dirt level detected: Slowing down for thorough cleaning');
            else
                next_move.speed_factor = 1.0;
            end
        end
        
        % Execute movement command
        function move(obj, next_move)
            % Update robot position in simulation
            obj.robot_actor.Translation = [next_move.x, next_move.y, 0.05];
            obj.robot_actor.Rotation = [next_move.theta, 0, 0];
            
            % Update internal state
            obj.position = [next_move.x, next_move.y, 0.05];
            obj.orientation = [next_move.theta, 0, 0];
            
            % Track energy usage based on movement
            speed = sqrt(next_move.x^2 + next_move.y^2);
            obj.energy_used = obj.energy_used + (0.1 * speed);
            
            % Update area cleaned
            obj.area_cleaned = obj.area_cleaned + (0.05 * speed); % Rough estimate
        end
        
        % Check for obstacles using sensors
        function obstacle_detected = check_for_obstacles(~)
            % Simulate obstacle detection
            % For demo, randomly detect obstacles
            obstacle_detected = (rand() > 0.9);
            
            if obstacle_detected
                disp('Obstacle detected!');
            end
        end
        
        % Implement obstacle avoidance behavior
        function obstacle_avoidance(obj)
            disp('Executing obstacle avoidance maneuver');
            
            % Simple avoidance: turn 90 degrees and continue
            current_theta = obj.orientation(1);
            new_theta = current_theta + 90;
            
            % Update orientation in simulation
            obj.robot_actor.Rotation = [new_theta, 0, 0];
            obj.orientation = [new_theta, 0, 0];
            
            % Increment collision counter (soft collision)
            obj.collision_count = obj.collision_count + 1;
            
            % Pause to visualize the avoidance
            pause(0.5);
        end
        
        % Check for unsafe areas (cliffs, etc.)
        function unsafe_detected = check_for_unsafe_areas(obj)
            % Check if robot is near a cliff/stairs
            unsafe_detected = false;
            
            % Check if near the stairs location
            stair_pos = [2, -2];
            dist_to_stairs = norm(obj.position(1:2) - stair_pos);
            
            if dist_to_stairs < 0.3
                unsafe_detected = true;
                disp('Cliff detected!');
            end
        end
        
        % Execute safety maneuver to avoid unsafe areas
        function safety_maneuver(obj)
            disp('Executing safety maneuver - backing away from cliff');
            
            % Move away from the cliff (reverse direction)
            current_x = obj.position(1);
            current_y = obj.position(2);
            
            % Move in opposite direction of stairs
            stair_pos = [2, -2];
            direction = [current_x, current_y] - stair_pos;
            direction = direction / norm(direction); % Normalize
            
            new_pos = [current_x, current_y] + 0.5 * direction;
            
            % Update position in simulation
            obj.robot_actor.Translation = [new_pos(1), new_pos(2), 0.05];
            obj.position = [new_pos(1), new_pos(2), 0.05];
            
            % Update orientation to face away from cliff
            angle = atan2(direction(2), direction(1)) * 180/pi;
            obj.robot_actor.Rotation = [angle, 0, 0];
            obj.orientation = [angle, 0, 0];
            
            % Pause to visualize the safety maneuver
            pause(0.5);
        end
        
        % Update performance metrics
        function update_performance_metrics(obj)
            % Most metrics are updated during other operations
            % This method could implement additional metrics or calculations
            
            % Calculate energy efficiency
            if obj.area_cleaned > 0
                energy_efficiency = obj.area_cleaned / obj.energy_used;
                disp(['Energy efficiency: ', num2str(energy_efficiency), ' m²/Wh']);
            end
        end
        
        % Update simulation display
        function update_display(~)
            % In a real implementation, this would update a visualization
            % For our simulation, the 3D view is updated automatically
            
            % Could add custom visualization elements here
        end
        
        % Return to charging dock
        function return_to_dock(obj)
            disp('Returning to charging dock...');
            
            % Simple direct path to dock
            % In a real implementation, this would use pathfinding
            
            % Get current position
            current_pos = obj.position(1:2);
            
            % Move toward dock in steps
            steps = 10;
            for i = 1:steps
                % Linear interpolation to dock
                alpha = i / steps;
                new_pos = (1-alpha) * current_pos + alpha * obj.dock_location;
                
                % Update position in simulation
                obj.robot_actor.Translation = [new_pos(1), new_pos(2), 0.05];
                obj.position = [new_pos(1), new_pos(2), 0.05];
                
                % Calculate direction to face
                direction = obj.dock_location - new_pos;
                angle = atan2(direction(2), direction(1)) * 180/pi;
                obj.robot_actor.Rotation = [angle, 0, 0];
                obj.orientation = [angle, 0, 0];
                
                % Update display
                pause(0.2);
            end
            
            % Final adjustment to dock perfectly
            obj.robot_actor.Translation = [obj.dock_location(1), obj.dock_location(2), 0.05];
            obj.position = [obj.dock_location(1), obj.dock_location(2), 0.05];
            
            obj.is_docked = true;
            disp('Successfully docked!');
        end
        
        % Calculate final performance metrics
        function calculate_final_metrics(obj)
            % Calculate final performance metrics at end of cleaning cycle
            
            % Time metrics
            total_time_minutes = obj.cleaning_time / 60;
            
            % Efficiency metrics
            if total_time_minutes > 0
                cleaning_rate = obj.area_cleaned / total_time_minutes; % m²/min
                obj.performance.cleaning_rate = cleaning_rate;
            end
            
            if obj.energy_used > 0
                energy_efficiency = obj.area_cleaned / obj.energy_used; % m²/Wh
                obj.performance.energy_efficiency = energy_efficiency;
            end
        end
        
        % Display performance report
        function display_performance_report(obj)
            disp('======= CLEANING CYCLE PERFORMANCE REPORT =======');
            disp(['Total area cleaned: ', num2str(obj.area_cleaned), ' m²']);
            disp(['Coverage percentage: ', num2str(obj.coverage_percentage), '%']);
            disp(['Cleaning time: ', num2str(obj.cleaning_time), ' seconds']);
            disp(['Energy used: ', num2str(obj.energy_used), ' Wh']);
            disp(['Collisions: ', num2str(obj.collision_count)]);
            disp(['Dirt collected: ', num2str(obj.dirt_collected), ' units']);
            disp(['Battery remaining: ', num2str(obj.battery_level), '%']);
            disp(['Bin level: ', num2str(obj.bin_level), '%']);
            disp('=================================================');
        end
    end
end