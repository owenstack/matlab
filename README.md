# Improved iRobot Create Controller for Webots

This project implements an advanced robotic vacuum cleaner controller for the iRobot Create model in the Webots simulation environment. The implementation follows the PEAS (Performance measures, Environment, Actuators, Sensors) framework to create a more efficient and intelligent cleaning robot.

## Key Features

### Intelligent Cleaning Patterns
- **Spiral Pattern**: The robot starts with an expanding spiral pattern to efficiently clean open areas
- **Wall Following**: Detects and follows walls to ensure edge cleaning
- **Obstacle Avoidance**: Smoothly navigates around furniture and other obstacles
- **Room Crossing**: Identifies uncleaned areas and navigates to them
- **Return-to-Dock**: Automatically returns to charging dock when battery is low

### Performance Tracking
- **Coverage Mapping**: Records and visualizes areas that have been cleaned
- **Path Planning**: Minimizes repeated coverage of the same areas
- **Efficiency Metrics**: Tracks performance including:
  - Coverage percentage
  - Distance traveled
  - Collisions
  - Energy consumption
  - Overall cleaning efficiency

## System Architecture

### Controller Components
- **State Machine**: Manages different operating modes (spiral, wall following, etc.)
- **Navigation System**: Tracks position and maintains a map of cleaned areas
- **Sensor Integration**: Processes data from multiple sensors for decision making
- **Battery Management**: Monitors energy usage and initiates return to dock

### Ground Supervisor
- Tracks and visualizes the robot's coverage
- Creates a heatmap showing cleaning intensity
- Logs performance metrics
- Provides a visual representation of the cleaning progress

## Code Structure

### improved_irobot_controller.c
The main robot controller implements:
- State machine for different cleaning behaviors
- Odometry for position tracking
- Advanced obstacle avoidance
- Battery management

Key components:
```c
typedef enum {
  STATE_SPIRAL,           // Spiral cleaning pattern
  STATE_WALL_FOLLOW,      // Follow walls to clean the perimeter
  STATE_OBSTACLE_AVOID,   // Avoid obstacles when detected
  STATE_RETURN_TO_DOCK,   // Return to charging dock
  STATE_SCANNING,         // Scan the environment
  STATE_CROSS_ROOM        // Move to uncleaned area
} RobotState;
```

The controller uses a cell-based map to track cleaned areas:
```c
static bool map[MAP_SIZE][MAP_SIZE];    // Simple occupancy grid (true = cleaned)
```

### improved_ground_supervisor.c
The supervisor provides:
- Real-time visualization of cleaning progress
- Performance metrics calculation
- Collision detection
- Path tracking and analysis

## Performance Measures

The controller addresses the PEAS requirements by optimizing:

1. **Cleanliness and Coverage**: 
   - Systematic coverage patterns ensure no areas are missed
   - Coverage percentage tracking

2. **Efficiency**:
   - Minimized repeated cleaning of the same areas
   - Energy consumption tracking
   - Optimized path planning

3. **Navigation**:
   - Smooth obstacle avoidance
   - Wall following for edge cleaning
   - Room-to-room navigation

4. **Safety**:
   - Cliff detection to prevent falls
   - Collision minimization
   - Gentle obstacle approach

5. **Autonomy**:
   - Battery management
   - Return-to-dock capability
   - Self-directed exploration

## How To Use

1. Copy the `improved_irobot_controller.c` and `improved_ground_supervisor.c` files to your Webots project
2. Configure the robot to use `improved_irobot_controller.c` as its controller
3. Set the world's supervisor to use `improved_ground_supervisor.c`
4. Run the simulation to see the improved cleaning performance

## Parameters for Tuning

Several parameters can be adjusted to optimize performance:
- `CELL_SIZE`: Resolution of the cleaning map
- `spiral_increment`: Controls how quickly the spiral expands
- `LOW_BATTERY_THRESHOLD`: When to return to dock

## Future Improvements

Potential enhancements for future versions:
1. Machine learning-based dirt detection
2. Multi-room mapping with doorway detection
3. Scheduled cleaning patterns
4. Floor type detection and cleaning mode adjustment

## Implementation of PEAS Framework

### Performance Measure
The system tracks multiple performance metrics:
- Coverage percentage (primary measure of cleaning effectiveness)
- Energy efficiency (area cleaned per energy unit)
- Cleaning time
- Number of collisions

### Environment
The controller is designed for indoor environments with:
- Multiple rooms
- Various obstacles
- Different floor types
- Potential cliffs (stairs)

### Actuators
The controller manages:
- Differential drive motors
- LED indicators
- (Simulated) vacuum and brush motors

### Sensors
The system utilizes:
- Bump sensors for collision detection
- Cliff sensors to detect stairs/ledges
- Wheel encoders for odometry
- (Simulated) battery level sensors

## Conclusion

This improved controller transforms the iRobot Create from a simple reactive cleaner to an intelligent, systematic cleaning system that efficiently covers areas while tracking its performance. The visualization tools provide valuable insights into the cleaning process and help identify areas for further optimization.