# Improved iRobot Create Controller for Webots

This project implements an advanced robotic vacuum cleaner controller for the iRobot Create model in the Webots simulation environment. The implementation follows the PEAS (Performance measures, Environment, Actuators, Sensors) framework to create a more efficient and intelligent cleaning robot.

## Key Features

### Intelligent Cleaning Patterns
- **Spiral Pattern**: The robot starts with an expanding spiral pattern for thorough initial cleaning
- **Line Pattern**: Switches to systematic parallel lines after spiral completion
- **Obstacle Avoidance**: Intelligently handles walls, furniture, and virtual boundaries
- **Smart Navigation**: Adapts movement patterns based on environment feedback

### Implementation Details

#### Cleaning States
```c
typedef enum {
    STATE_SPIRAL,
    STATE_LINES,
    STATE_OBSTACLE_AVOIDANCE
} CleaningState;
```

### Movement Patterns

#### Spiral Pattern
- Starts with a small radius (0.2m) that gradually expands
- Uses differential wheel speeds to create spiral motion
- Transitions to line pattern when spiral radius exceeds 2.0m
- Efficient for covering open areas quickly

#### Line Pattern
- Systematic back-and-forth movement with configurable line spacing
- Alternates direction to ensure complete coverage
- Maintains parallel lines with precise turns
- Line length of 4.0m with 0.3m spacing between lines

### Obstacle Handling
- **Collision Detection**: Uses both left and right bumper sensors
- **Cliff Detection**: Four cliff sensors prevent falls
- **Virtual Wall Detection**: Respects IR-based virtual boundaries
- **Smart Recovery**: 
  - Backs away from obstacles
  - Turns in opposite direction of collision
  - Resumes previous cleaning pattern

### Technical Implementation

#### Sensors Integration
- Bumper sensors (left, right)
- Cliff sensors (left, front-left, front-right, right)
- Wheel position sensors for odometry
- IR receiver for virtual wall detection

#### Motion Control
- Precision turning using wheel encoders
- Speed control for various movement patterns
- Smooth transitions between states

### Ground Supervisor
- Tracks robot position in real-time
- Visualizes cleaning coverage
- Uses Display device for coverage visualization
- Updates cleaning progress dynamically

## Code Structure

### Robot Controller (create_avoid_obstacles.c)
- State machine implementation
- Sensor integration
- Movement pattern algorithms
- Obstacle avoidance logic

### Ground Supervisor (ground.c)
- Position tracking
- Coverage visualization
- Real-time progress monitoring

## Usage

1. Open the simulation in Webots
2. The robot automatically starts in spiral cleaning mode
3. Transitions to line pattern after initial spiral coverage
4. Avoids obstacles and adjusts patterns as needed

## Parameters for Tuning

Several parameters can be adjusted to optimize performance:
```c
static double spiral_radius = 0.2;        // Initial spiral radius
static const double LINE_SPACING = 0.3;   // Space between parallel lines
#define MAX_SPEED 16                      // Maximum wheel speed
#define HALF_SPEED 8                      // Speed for turning
```

## Future Improvements

Potential enhancements for future versions:
1. Battery level monitoring and charging behavior
2. Dynamic pattern adjustment based on room size
3. Dirt detection and focused cleaning
4. Multiple room navigation
5. Zone-based cleaning priorities