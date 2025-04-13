/*
 * Description:  Default controller of the iRobot Create robot
 */

/* include headers */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

/* Cleaning states */
typedef enum {
    STATE_SPIRAL,
    STATE_LINES,
    STATE_OBSTACLE_AVOIDANCE
} CleaningState;

static CleaningState current_state = STATE_SPIRAL;
static double spiral_radius = 0.2;  // Initial spiral radius
static double line_direction = 1.0;  // Direction for parallel lines (1.0 or -1.0)
static double current_line_position = 0.0;
static const double LINE_SPACING = 0.3;  // Space between parallel lines
static double total_distance = 0.0;

/* device stuff */
#define BUMPERS_NUMBER 2
#define BUMPER_LEFT 0
#define BUMPER_RIGHT 1
static WbDeviceTag bumpers[BUMPERS_NUMBER];
static const char *bumpers_name[BUMPERS_NUMBER] = {"bumper_left", "bumper_right"};

#define CLIFF_SENSORS_NUMBER 4
#define CLIFF_SENSOR_LEFT 0
#define CLIFF_SENSOR_FRONT_LEFT 1
#define CLIFF_SENSOR_FRONT_RIGHT 2
#define CLIFF_SENSOR_RIGHT 3
static WbDeviceTag cliff_sensors[CLIFF_SENSORS_NUMBER];
static const char *cliff_sensors_name[CLIFF_SENSORS_NUMBER] = {"cliff_left", "cliff_front_left", "cliff_front_right",
                                                               "cliff_right"};

#define LEDS_NUMBER 3
#define LED_ON 0
#define LED_PLAY 1
#define LED_STEP 2
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_name[LEDS_NUMBER] = {"led_on", "led_play", "led_step"};

static WbDeviceTag receiver;
static const char *receiver_name = "receiver";

WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

/* Misc Stuff */
#define MAX_SPEED 16
#define NULL_SPEED 0
#define HALF_SPEED 8
#define MIN_SPEED -16

#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
#define ENCODER_RESOLUTION 507.9188

static bool moving_forward = true;  // Global state for line pattern movement

/* helper functions */
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_devices() {
  int i;

  receiver = wb_robot_get_device(receiver_name);
  wb_receiver_enable(receiver, get_time_step());

  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_name[i]);

  for (i = 0; i < BUMPERS_NUMBER; i++) {
    bumpers[i] = wb_robot_get_device(bumpers_name[i]);
    wb_touch_sensor_enable(bumpers[i], get_time_step());
  }

  for (i = 0; i < CLIFF_SENSORS_NUMBER; i++) {
    cliff_sensors[i] = wb_robot_get_device(cliff_sensors_name[i]);
    wb_distance_sensor_enable(cliff_sensors[i], get_time_step());
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, get_time_step());
  wb_position_sensor_enable(right_position_sensor, get_time_step());
}

static bool is_there_a_collision_at_left() {
  return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0);
}

static bool is_there_a_collision_at_right() {
  return (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}

static void fflush_ir_receiver() {
  while (wb_receiver_get_queue_length(receiver) > 0)
    wb_receiver_next_packet(receiver);
}

static bool is_there_a_virtual_wall() {
  return (wb_receiver_get_queue_length(receiver) > 0);
}

static bool is_there_a_cliff_at_left() {
  return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_LEFT]) < 100.0 ||
          wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_LEFT]) < 100.0);
}

static bool is_there_a_cliff_at_right() {
  return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_RIGHT]) < 100.0 ||
          wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT]) < 100.0);
}

static bool is_there_a_cliff_at_front() {
  return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_LEFT]) < 100.0 ||
          wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT]) < 100.0);
}

static void go_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

static void go_backward() {
  wb_motor_set_velocity(left_motor, -HALF_SPEED);
  wb_motor_set_velocity(right_motor, -HALF_SPEED);
}

static void stop() {
  wb_motor_set_velocity(left_motor, NULL_SPEED);
  wb_motor_set_velocity(right_motor, NULL_SPEED);
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void turn(double angle) {
  stop();
  double l_offset = wb_position_sensor_get_value(left_position_sensor);
  double r_offset = wb_position_sensor_get_value(right_position_sensor);
  step();
  double neg = (angle < 0.0) ? -1.0 : 1.0;
  wb_motor_set_velocity(left_motor, neg * HALF_SPEED);
  wb_motor_set_velocity(right_motor, -neg * HALF_SPEED);
  double orientation;
  do {
    double l = wb_position_sensor_get_value(left_position_sensor) - l_offset;
    double r = wb_position_sensor_get_value(right_position_sensor) - r_offset;
    double dl = l * WHEEL_RADIUS;                 // distance covered by left wheel in meter
    double dr = r * WHEEL_RADIUS;                 // distance covered by right wheel in meter
    orientation = neg * (dl - dr) / AXLE_LENGTH;  // delta orientation in radian
    step();
  } while (orientation < neg * angle);
  stop();
  step();
}

static void set_wheel_speeds(double left_speed, double right_speed) {
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
}

static void move_in_spiral() {
  // Calculate speeds for spiral motion
  double base_speed = MAX_SPEED * 0.8;
  double speed_diff = (spiral_radius * 0.2); // Difference increases with radius
  
  // Inner wheel moves slower than outer wheel to create spiral
  set_wheel_speeds(base_speed - speed_diff, base_speed);
  
  // Gradually increase spiral radius
  spiral_radius += 0.001;
  
  // If spiral gets too large, switch to line pattern
  if (spiral_radius > 2.0) {
    current_state = STATE_LINES;
    total_distance = 0.0; // Reset distance for line pattern
    moving_forward = true; // Initialize the line pattern state
    printf("Switching to line pattern\n");
  }
}

static void move_in_lines() {
  double target_distance = 4.0; // Length of each line
  
  if (moving_forward) {
    go_forward();
    total_distance += (MAX_SPEED * WHEEL_RADIUS * get_time_step() / 1000.0); // More accurate distance calculation
    
    if (total_distance >= target_distance) {
      moving_forward = false;
      total_distance = 0.0;
      
      // Complete the turn sequence
      turn(M_PI_2 * line_direction);
      go_forward();
      passive_wait(1.0);
      turn(M_PI_2 * line_direction);
      
      // Reset for next line
      moving_forward = true;
      line_direction *= -1.0;
      current_line_position += LINE_SPACING;
    }
  }
}

static bool handle_obstacles() {
  if (is_there_a_virtual_wall() || 
      is_there_a_collision_at_left() || 
      is_there_a_collision_at_right() || 
      is_there_a_cliff_at_left() || 
      is_there_a_cliff_at_right() || 
      is_there_a_cliff_at_front()) {
    
    current_state = STATE_OBSTACLE_AVOIDANCE;
    go_backward();
    passive_wait(0.5);
    
    bool collision_left = is_there_a_collision_at_left();
    bool cliff_left = is_there_a_cliff_at_left();

    if (collision_left || cliff_left) {
      turn(M_PI_2); // Turn right
    } else {
      turn(-M_PI_2); // Turn left
    }
    
    return true;
  }
  return false;
}

/* main */
int main(int argc, char **argv) {
  wb_robot_init();

  printf("Smart cleaning robot controller started...\n");

  init_devices();
  srand(time(NULL));

  wb_led_set(leds[LED_ON], true);
  passive_wait(0.5);

  while (true) {
    // First check for obstacles
    if (!handle_obstacles()) {
      // If no obstacles, execute current cleaning pattern
      switch (current_state) {
        case STATE_SPIRAL:
          move_in_spiral();
          break;
        case STATE_LINES:
          move_in_lines();
          break;
        case STATE_OBSTACLE_AVOIDANCE:
          // Return to previous pattern after obstacle is cleared
          current_state = (spiral_radius > 2.0) ? STATE_LINES : STATE_SPIRAL;
          break;
      }
    }
    
    fflush_ir_receiver();
    step();
  }

  return EXIT_SUCCESS;
}
